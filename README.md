# ZMK Analog Stick Driver

Multi-mode analog stick driver for ZMK firmware. Reads analog position sensors (TMR, hall-effect, etc.) via Zephyr ADC `io-channels` and supports three runtime modes switchable per keymap layer:

- **Switch mode** — Joystick acts as 4 (or 8 diagonal) directional keys with configurable threshold and hysteresis
- **Mouse pointer mode** — Distance from center controls cursor speed with acceleration ramp
- **Mouse scroll mode** — Distance from center controls scroll speed

## Features

- Layer-aware mode switching via ZMK keymap (e.g., arrows on layer 0, mouse on layer 1, scroll on layer 2)
- Works with any Zephyr-supported ADC (MCP3208, nRF52840 SAADC, etc.) via `io-channels`
- Split keyboard support (peripheral → central → host)
- IIR biquad filter for noise reduction
- Per-axis linear deadzone
- Adaptive idle/active poll rates for power savings
- Pulse-read sensor power gating via GPIO with auto-disable at high duty cycle
- Q16.16 fixed-point math on all hot paths
- Up to 8 joysticks (16 sensors) per keyboard half

## Installation

Add to your `zmk-config/config/west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: wdtamagi
      url-base: https://github.com/wdtamagi
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-driver-analog-stick
      remote: wdtamagi
      revision: main
  self:
    path: config
```

## Devicetree Configuration

### MCP3208 SPI ADC Setup

```dts
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 9 GPIO_ACTIVE_LOW>,
               <&gpio0 6 GPIO_ACTIVE_LOW>;

    mcp3208_0: mcp3208@0 {
        compatible = "microchip,mcp3208";
        reg = <0>;
        spi-max-frequency = <1000000>;
        #io-channel-cells = <1>;
    };

    mcp3208_1: mcp3208@1 {
        compatible = "microchip,mcp3208";
        reg = <1>;
        spi-max-frequency = <1000000>;
        #io-channel-cells = <1>;
    };
};
```

### Analog Stick Instances

```dts
#include <dt-bindings/analog-stick/modes.h>

/ {
    /* Dual-axis joystick on MCP3208 channels 0,1 */
    analog_stick0: analog_stick0 {
        compatible = "zmk,analog-stick";
        io-channels = <&mcp3208_0 0>, <&mcp3208_0 1>;
        x-min = <200>;
        x-center = <2048>;
        x-max = <3900>;
        y-min = <200>;
        y-center = <2048>;
        y-max = <3900>;
        deadzone = <50>;
        wait-period-idle = <100>;
        wait-period-active = <1>;
    };

    /* Second joystick on MCP3208 channels 2,3 */
    analog_stick1: analog_stick1 {
        compatible = "zmk,analog-stick";
        io-channels = <&mcp3208_0 2>, <&mcp3208_0 3>;
        x-min = <200>;
        x-center = <2048>;
        x-max = <3900>;
        y-min = <200>;
        y-center = <2048>;
        y-max = <3900>;
        deadzone = <50>;
    };
};
```

### Layer-Aware Input Listener

```dts
#include <dt-bindings/analog-stick/modes.h>
#include <dt-bindings/zmk/keys.h>

/ {
    analog_stick_listener0: analog_stick_listener0 {
        compatible = "zmk,analog-stick-listener";
        device = <&analog_stick0>;

        /* Layer 0: Arrow keys */
        layer0 {
            layers = <0>;
            mode = <ANALOG_STICK_MODE_SWITCH>;
            switch-activation-point = <40>;
            switch-hysteresis = <4>;
            up-keycode = <UP>;
            down-keycode = <DOWN>;
            left-keycode = <LEFT>;
            right-keycode = <RIGHT>;
        };

        /* Layer 1: Mouse pointer */
        layer1 {
            layers = <1>;
            mode = <ANALOG_STICK_MODE_MOUSE>;
            mouse-min-speed = <1>;
            mouse-max-speed = <20>;
        };

        /* Layer 2: Scroll wheel */
        layer2 {
            layers = <2>;
            mode = <ANALOG_STICK_MODE_SCROLL>;
            scroll-divisor = <4>;
        };
    };
};
```

### Split Keyboard Setup

On the **peripheral** side (each half):

```dts
/ {
    analog_stick_split0: analog_stick_split0 {
        compatible = "zmk,analog-stick-split";
        device = <&analog_stick0>;
        reg = <0>;
    };
};
```

On the **central** side (dongle):

```dts
/ {
    analog_stick_split0: analog_stick_split0 {
        compatible = "zmk,analog-stick-split";
        reg = <0>;  /* must match peripheral reg */
    };

    /* Listener on central references the split proxy device */
    analog_stick_listener0: analog_stick_listener0 {
        compatible = "zmk,analog-stick-listener";
        device = <&analog_stick_split0>;
        /* ... layer configs ... */
    };
};
```

### Power Management (Pulse-Read)

For battery savings, toggle sensor power per read cycle:

```dts
analog_stick0: analog_stick0 {
    compatible = "zmk,analog-stick";
    io-channels = <&mcp3208_0 0>, <&mcp3208_0 1>;
    /* ... calibration ... */
    enable-gpios = <&gpio0 10 GPIO_ACTIVE_HIGH>;
    pulse-read;
    read-turn-on-time = <100>;  /* 100µs settling (TMR default) */
};
```

> **Hall-effect users:** Set `read-turn-on-time = <5000>` (or your sensor's required value). The default 100µs is optimized for TMR sensors.

When the active poll rate makes pulse-read counterproductive (settling time > half the poll period), the driver automatically keeps sensors powered and logs a message. Pulse-read re-engages when the stick returns to idle.

### Ratiometric Deadzone

TMR sensors have a temperature coefficient of roughly −0.1 to −0.3 %/°C on bridge output. Over a 40 °C swing a 12-bit sensor centered at 2048 can drift ~164 LSB, overwhelming a fixed 50-count deadzone and producing phantom stick activity at elevated temperature.

The `deadzone-percent` property expresses the deadzone as a percentage of the sensor's smaller calibrated half-range, so the effective deadzone in raw units scales with the actual output swing:

```
half_range   = min(x_center - x_min, x_max - x_center)
effective_dz = (half_range × deadzone-percent) / 100
```

The computation runs once at init and is stored in `effective_deadzone`. With a typical 12-bit stick (half-range ~1848) and the default 5%, the effective deadzone is ~92 counts — comparable to the legacy default of 50, but temperature-stable.

Use `deadzone-percent` for new designs:

```dts
analog_stick0: analog_stick0 {
    compatible = "zmk,analog-stick";
    /* ... calibration ... */
    deadzone-percent = <5>;  /* 5% of half-range (default; omit to use this value) */
};
```

If both `deadzone` and `deadzone-percent` are present in the overlay, `deadzone-percent` takes precedence and a warning is logged at init.

### Filtering

Apply a 2nd-order Butterworth low-pass IIR filter to reduce sensor noise:

```dts
analog_stick0: analog_stick0 {
    compatible = "zmk,analog-stick";
    /* ... other properties ... */
    /* 25Hz cutoff at 125Hz sample rate (8ms poll) */
    filter-coefficients = <0x3D8A3D71 0x3E0A3D71 0x3D8A3D71
                           0x3F800000 0xBF927DBD 0x3ED32C9C>;
};
```

The six values are IEEE-754 uint32 hex representations of biquad coefficients (b0, b1, b2, a0, a1, a2). The example above provides a 25Hz cutoff suitable for the TMR noise profile at an 8ms active poll rate. **Coefficients must be recalculated for poll rates other than 8ms.** The cutoff frequency can be computed using the bilinear transform:

```
fc = desired cutoff (Hz), fs = 1000 / wait-period-active (Hz)
```

### Voltage Matching

Both TMR and hall-effect sensors output a ratiometric voltage (output is a fraction of VDD). For accurate readings:

- **MCP3208**: Sensor VDD must equal ADC VREF. The MCP3208 uses VREF as its full-scale reference. If VDD ≠ VREF, the output can exceed the ADC input range, causing clipping.
- **nRF52840 SAADC**: The internal reference is 0.6V with an optional 1/6 gain prescaler (3.6V effective range). For 3.3V sensors, use the 1/6 prescaler. If sensor VDD exceeds the effective range, add a resistive voltage divider on the sensor output or adjust the SAADC gain setting in your board's DTS.

### SAADC Oversampling

For nRF52840 SAADC users, 4× or 8× oversampling significantly reduces noise at the cost of longer conversion time:

```dts
&adc {
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,oversampling = <4>;  /* 4× oversampling */
        /* ... other channel config ... */
    };
};
```

The `oversampling` property is on the SAADC channel node, not on the analog stick driver node. Each doubling of oversampling doubles conversion time (~36µs per sample at 4× on nRF52840). With 8 joysticks this adds to the per-scan budget, but remains well within the BLE poll floor.

## Calibration

To determine `x-min`, `x-center`, `x-max` (and Y equivalents):

1. Enable debug logging: `CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL_DBG=y`
2. With the stick at rest, note the raw ADC value → `x-center`
3. Push the stick fully left/up, note the value → `x-min` (use the lower value)
4. Push the stick fully right/down, note the value → `x-max` (use the higher value)
5. Add ~5% margin inside the min/max to account for mechanical variation

Typical 12-bit MCP3208 values: min ~200, center ~2048, max ~3900.

## Kconfig Options

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_ZMK_ANALOG_STICK` | y (auto) | Enable the driver (auto-enabled by DT) |
| `CONFIG_ZMK_ANALOG_STICK_WORK_QUEUE_STACK_SIZE` | 2560 | Work queue stack size |
| `CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR` | 8 | Min active poll interval (ms) when BLE enabled |
| `CONFIG_ZMK_ANALOG_STICK_INIT_PRIORITY` | 90 | Device init priority (POST_KERNEL) |

## Properties Reference

### zmk,analog-stick

| Property | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| `io-channels` | phandle-array | yes | — | ADC channels (idx 0=X, idx 1=Y optional) |
| `x-min` | int | yes | — | Raw ADC at min X deflection |
| `x-center` | int | yes | — | Raw ADC at X rest position |
| `x-max` | int | yes | — | Raw ADC at max X deflection |
| `y-min` | int | no | 0 | Raw ADC at min Y deflection |
| `y-center` | int | no | 0 | Raw ADC at Y rest position |
| `y-max` | int | no | 0 | Raw ADC at max Y deflection |
| `deadzone` | int | no | 50 | Per-axis linear deadzone in raw ADC units (deprecated; prefer `deadzone-percent`) |
| `deadzone-percent` | int | no | 5 | Deadzone as % of smaller calibrated half-range; takes precedence over `deadzone` |
| `invert-x` | boolean | no | false | Negate X axis |
| `invert-y` | boolean | no | false | Negate Y axis |
| `wait-period-idle` | int | no | 100 | Idle poll interval (ms) |
| `wait-period-active` | int | no | 1 | Active poll interval (ms) |
| `enable-gpios` | phandle-array | no | — | Sensor power GPIO |
| `pulse-read` | boolean | no | false | Toggle power per read |
| `read-turn-on-time` | int | no | 100 | Settling time (µs); TMR default, set 1000-5000 for hall-effect |
| `inter-channel-settling-time` | int | no | 40 | Inter-channel ADC settling (µs); 40 for SAADC, 5 for MCP3208 |
| `filter-coefficients` | array | no | — | 6 IEEE-754 uint32 biquad coefficients |

### zmk,analog-stick-listener

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| `device` | phandle | yes | Analog stick device to listen to |

**Child node properties (per layer):**

| Property | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| `layers` | array | yes | — | Layer indices |
| `mode` | int | yes | — | Mode constant (0=switch, 1=mouse, 2=scroll) |
| `switch-activation-point` | int | no | 40 | Switch mode threshold [0-127] |
| `switch-hysteresis` | int | no | 4 | Hysteresis for switch deactivation |
| `up-keycode` | int | no | 0 | Up key (switch mode) |
| `down-keycode` | int | no | 0 | Down key (switch mode) |
| `left-keycode` | int | no | 0 | Left key (switch mode) |
| `right-keycode` | int | no | 0 | Right key (switch mode) |
| `mouse-min-speed` | int | no | 1 | Min cursor speed (mouse mode) |
| `mouse-max-speed` | int | no | 15 | Max cursor speed (mouse mode) |
| `scroll-divisor` | int | no | 4 | Scroll rate divisor (scroll mode) |

## Known Limitations

**BLE bandwidth with 5+ sticks**: The per-event split transport sends one BLE notification per
input axis event. Configurations with 5 or more sticks will approach the BLE throughput ceiling
at short connection intervals, resulting in increased input latency. A custom GATT transport
(`packed-ble-gatt-transport`) is planned to address this by coalescing all axis changes into one
notification per scan cycle.

## Migration Notes

### From initial release to phase 1+2

- **`filter-coefficients`** type changed from `uint8-array` to `array`. Update values from byte arrays to IEEE-754 uint32 hex integers.
- **`switch-activation-point`** and **`switch-hysteresis`** moved from the driver node (`zmk,analog-stick`) to the listener's per-layer child node (`zmk,analog-stick-listener` child). Move these properties in your overlay.
- **`read-turn-on-time`** default changed from 5000µs to 100µs (optimized for TMR sensors). Hall-effect users with `pulse-read` enabled must add `read-turn-on-time = <5000>` (or their sensor's required value) to their overlay.

## License

MIT
