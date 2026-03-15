# ZMK Analog Stick Driver

Multi-mode analog stick driver for ZMK firmware. Reads hall-effect sensors via Zephyr ADC `io-channels` and supports three runtime modes switchable per keymap layer:

- **Switch mode** — Joystick acts as 4 (or 8 diagonal) directional keys with configurable threshold and hysteresis
- **Mouse pointer mode** — Distance from center controls cursor speed with acceleration ramp
- **Mouse scroll mode** — Distance from center controls scroll speed

## Features

- Layer-aware mode switching via ZMK keymap (e.g., arrows on layer 0, mouse on layer 1, scroll on layer 2)
- Works with any Zephyr-supported ADC (MCP3208, nRF52840 SAADC, etc.) via `io-channels`
- Split keyboard support (peripheral → central → host)
- IIR biquad filter for noise reduction
- Circular deadzone for uniform mouse behavior
- Adaptive idle/active poll rates for power savings
- Pulse-read sensor power gating via GPIO
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
        switch-activation-point = <40>;
        switch-hysteresis = <4>;
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
    read-turn-on-time = <3000>;  /* 3ms settling */
};
```

### Optional Biquad Filter

Apply a low-pass IIR filter to reduce ADC noise:

```dts
analog_stick0: analog_stick0 {
    compatible = "zmk,analog-stick";
    /* ... other properties ... */
    filter-coefficients = [
        /* b0, b1, b2, a0(=1.0), a1, a2 as IEEE-754 bytes */
        /* Example: 20Hz low-pass at 125Hz sample rate */
        3D B1 4D A4  3D B1 4D A4  00 00 00 00
        3F 80 00 00  BE DA 1B 71  3E 2C 68 05
    ];
};
```

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
| `CONFIG_ZMK_ANALOG_STICK_WORK_QUEUE_STACK_SIZE` | 2048 | Work queue stack size |
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
| `deadzone` | int | no | 50 | Deadzone in raw ADC units |
| `invert-x` | boolean | no | false | Negate X axis |
| `invert-y` | boolean | no | false | Negate Y axis |
| `wait-period-idle` | int | no | 100 | Idle poll interval (ms) |
| `wait-period-active` | int | no | 1 | Active poll interval (ms) |
| `enable-gpios` | phandle-array | no | — | Sensor power GPIO |
| `pulse-read` | boolean | no | false | Toggle power per read |
| `read-turn-on-time` | int | no | 5000 | Settling time (µs) |
| `switch-activation-point` | int | no | 40 | Switch mode threshold |
| `switch-hysteresis` | int | no | 0 | Hysteresis band (0 = 10% of activation) |
| `filter-coefficients` | uint8-array | no | — | 6 IEEE-754 biquad coefficients |

### zmk,analog-stick-listener

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| `device` | phandle | yes | Analog stick device to listen to |

**Child node properties (per layer):**

| Property | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| `layers` | array | yes | — | Layer indices |
| `mode` | int | yes | — | Mode constant (0=switch, 1=mouse, 2=scroll) |
| `up-keycode` | int | no | 0 | Up key (switch mode) |
| `down-keycode` | int | no | 0 | Down key (switch mode) |
| `left-keycode` | int | no | 0 | Left key (switch mode) |
| `right-keycode` | int | no | 0 | Right key (switch mode) |
| `mouse-min-speed` | int | no | 1 | Min cursor speed (mouse mode) |
| `mouse-max-speed` | int | no | 15 | Max cursor speed (mouse mode) |
| `scroll-divisor` | int | no | 4 | Scroll rate divisor (scroll mode) |

## License

MIT
