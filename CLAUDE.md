# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A ZMK firmware Zephyr module providing a multi-mode analog stick driver. It reads analog position sensors (TMR, hall-effect) via Zephyr ADC `io-channels` and supports three runtime modes switchable per keymap layer: switch (directional keys), mouse pointer, and mouse scroll. Targets split keyboards with up to 8 joysticks per half.

## Build

This is a Zephyr module, not a standalone project. It is consumed via west manifest by a `zmk-config` repo. There is no local build command — the module is built as part of a ZMK firmware build:

```sh
west build -b <board> -- -DZMK_CONFIG=/path/to/zmk-config/config
```

Enable debug logging with `CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL_DBG=y` in the ZMK config's `.conf` file.

There are no tests or linting commands in this repo.

## Architecture

### Data Flow

```
ADC hardware
  → analog_stick.c (read, filter, rescale, emit INPUT_ABS events)
  → input_listener.c (layer-aware mode dispatch → switch/mouse/scroll)
  → hid_accumulator.c (sum all sticks → one HID report)
  → ZMK HID endpoint
```

On split keyboards, `input_split.c` inserts between the driver and listener:
```
Peripheral: analog_stick → input_split (forward over BLE)
Central:    input_split (proxy device re-emits) → input_listener → HID
```

### Scan Coordinator (`scan_coordinator.c`)

Central orchestrator that replaces per-stick work items. Runs on a shared dedicated work queue (`analog_stick_wq`) at cooperative priority 7.

- **GPIO grouping**: Sticks sharing the same enable GPIO are grouped; one settling wait per group.
- **Batched scan cycle**: Phase 1 reads all ADC channels, Phase 2 processes/emits, Phase 3 flushes HID, Phase 4 selects next poll rate.
- **Adaptive polling**: Switches between `wait-period-idle` (default 100ms) and `wait-period-active` (default 1ms) based on stick deflection.
- **BLE CI tracking**: On non-central BLE devices, tracks negotiated connection interval and enforces a poll floor (`CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR`, default 8ms).
- **Pulse-read suppression**: Auto-disables pulse-read when settling time exceeds half the poll period; re-enables on return to idle.
- Initializes at `APPLICATION` priority via `SYS_INIT`, after stick devices init at `POST_KERNEL`.

### Analog Stick Driver (`analog_stick.c`)

Per-instance Zephyr device driver (`zmk,analog-stick` compatible). Key internals:

- **Q16.16 fixed-point** (`q16.h`): All hot-path math uses saturating Q16.16 arithmetic — no floating point at runtime. Float is only used at init to parse filter coefficients.
- **IIR biquad filter**: Direct Form II Transposed in Q16.16. Normalizes ADC values to [0,1] before filtering, denormalizes after. Primed with 20 iterations at init. Resets state on saturation.
- **Rescaling pipeline**: `raw ADC → deadzone removal → normalize to [-1,1] via precomputed reciprocals → clamp → optional invert → scale to [-127, 127]`.
- **Rail detection**: Clamps to center if ADC reads within 15 LSB of rail (prevents noise-induced false extremes).
- **Ratiometric deadzone**: `deadzone-percent` computes effective deadzone from the smaller calibrated half-range at init. Takes precedence over legacy `deadzone` property.
- Events emitted every tick while the stick is deflected (not just on change), so mouse/scroll modes get continuous input.

### Input Listener (`input_listener.c`)

Per-listener instance (`zmk,analog-stick-listener` compatible). Subscribes to `INPUT_ABS` events from its device.

- **Layer resolution**: On each sync event, scans layer configs to find the highest active layer with a matching config. Mode transitions trigger cleanup (release pressed keys, reset accumulators).
- **Switch mode**: Directions map to keymap positions via `zmk_keymap_position_state_changed()` — these are real matrix positions, enabling ZMK Studio editing and any ZMK behavior.
- **Mouse mode**: Sub-pixel Q16.16 accumulator with linear speed ramp. Fast integer magnitude approximation (no sqrt).
- **Scroll mode**: Q16.16 tick accumulator with configurable divisor. Y axis negated for natural scroll.
- **HID accumulator** (`hid_accumulator.c`): Sums mouse/scroll deltas from all sticks, emits one combined `zmk_hid_mouse_*` report per flush. Thread-safe by single-queue contract.

### Zephyr Version Compatibility

The listener and split modules detect Zephyr 3.5 vs 4.x via `__has_include(<zephyr/input/input_callback.h>)` to handle the 2-arg vs 3-arg `INPUT_CALLBACK_DEFINE` signature. The driver uses `DT_PROP_OR` patterns for optional Y-axis properties to avoid token-pasting errors on Zephyr 3.5.

### Devicetree Bindings

Three compatibles in `dts/bindings/`:
- `zmk,analog-stick` — the hardware driver (ADC channels, calibration, filter, power management)
- `zmk,analog-stick-listener` — mode-aware input processor with per-layer child nodes
- `zmk,analog-stick-split` — split keyboard transport proxy (peripheral forwards, central re-emits)

### Key Constraints

- `BUILD_ASSERT` requires arithmetic right-shift for signed integers (both `int` and `int64_t`).
- Filter coefficient a0 must be exactly 1.0 (normalized biquad). Jury stability criterion enforced at init.
- `CONFIG_ZMK_ANALOG_STICK_STICK_COUNT` must match the number of DTS stick nodes (sizes the static registry).
- Switch mode `source=0` assumes listener runs on central side only.
