# DSPico

Inline digital **parametric-EQ USB bridge** for the Waveshare **RP2350-USB-CM**.

```
PC / phone ──USB──► [ DSPico = RP2350 ] ──USB──► USB DAC
                      applies PEQ (digital)
```

The bridge appears to the PC/phone as a driverless **USB Audio Class 2 (UAC2)
output** (48 kHz / 24-bit stereo), and re-transmits the (eventually EQ'd) stream
as a **USB host** to a downstream USB DAC. See `docs`/the build brief for the
full design.

---

## ⚠️ Current status: Phase 0 + Phase 1 only

Per the build brief (§8, §11) this repository stops at the **Phase 1b hardware
go/no-go gate**. The one unproven assumption — *can we stream isochronous USB
audio as a host over the bit-banged PIO port?* — must be validated on real
hardware with a logic analyzer **before** the rest of the bridge is built.

What is implemented here:

| Phase | What it does | Where |
|-------|--------------|-------|
| **0** | Dual-stack skeleton: native USB **device** on core0 + PIO-USB **host** on core1, 120 MHz clock, correct PIO pin polarity | `src/main.c`, `src/board_config.h` |
| **1a** | Native side enumerates as a **UAC2 output** — 48 kHz/24-bit stereo, **no capture**, Feature Unit with **Volume + Mute**; received audio is accepted and discarded | `src/usb_descriptors.*`, `src/uac2_device.c` |
| **1b** | PIO **host** enumerates one class-compliant DAC, sets 48 kHz, opens its **isochronous OUT** endpoint, and streams a firmware-generated **1 kHz test tone** | `src/uac_host.c`, `src/test_tone.c` |

**Not yet built** (deliberately): the PC→DAC passthrough (Phase 2), the biquad
PEQ + pre-gain + host-volume DSP (Phase 3), the WebUSB configurator (Phase 3),
clock-domain feedback tuning (Phase 4), and robustness/hot-plug (Phase 5).

> Honesty note: this firmware could not be compiled in the authoring
> environment (no ARM toolchain / SDK access there). It is written against the
> TinyUSB bundled with **Pico SDK 2.1.1** and Pico-PIO-USB `master`. Expect to
> compile it during Phase 1a bring-up and iron out any version-specific macro
> details — the UAC2 descriptor macros in particular are version-sensitive, and
> the iso-over-PIO data path in `uac_host.c` is the explicit experimental risk.

---

## Hardware mods — see [`docs/hardware-fix.md`](docs/hardware-fix.md)

> **Golden rule: don't mod the board until the software tells you to.** Flash
> Phase 0 first. The PIO port often works as-shipped — only do the mod below
> **if the host port fails to enumerate the DAC.**

The PIO port (Type-C2) ships "device-leaning": a 1.5 kΩ pull-up on D+ → 3V3,
which fights the DAC's pull-up and corrupts the idle line when we act as host.

1. If (and only if) host enumeration fails: **remove the 1.5 kΩ D+ pull-up.**
   Waveshare swaps this resistor's designator between board variants, so
   **trust the net, not the label** — remove whichever ~1.5 kΩ part sits between
   the Type-C2 **D+ (GPIO13)** line and **3V3**, after confirming with a meter:

   | Board variant | 1.5 kΩ D+ pull-up to remove | Empty footprint |
   |---------------|-----------------------------|-----------------|
   | **RP2350-USB-C**  | **R13** | R10 (NC) |
   | RP2350-USB-CM     | **R10** | R13 (NC) |

2. Still flaky / misses hot-plug? Add **~15 kΩ pull-downs** on D+ → GND and
   D− → GND.
3. Use a **self-powered DAC** — there is no VBUS switch/limit on the PIO port.
   A **captive or Type-A-cabled** DAC also avoids Type-C CC negotiation issues.
4. Leave the 27 Ω series resistors and the CC pull-downs alone — they're correct.

The firmware already handles the reversed pin order (D− = GPIO12, D+ = GPIO13)
via `DSPICO_PIO_USB_PINOUT_DPDM_SWAP` in `src/board_config.h` — this is config,
not a solder fix, and is identical for both the -C and -CM variants.

---

## Build

Prerequisites: `arm-none-eabi-gcc`, `cmake` (≥3.13), and the **Pico SDK**
(export `PICO_SDK_PATH`, or use `-DPICO_SDK_FETCH_FROM_GIT=ON`).

```bash
./setup.sh                                   # clones lib/Pico-PIO-USB
export PICO_SDK_PATH=/path/to/pico-sdk       # if not already set
cmake -B build -DPICO_BOARD=pico2
cmake --build build -j
# -> build/dspico.uf2
```

## Flash

Hold **BOOTSEL**, tap **RUN/reset**, release BOOTSEL → the board mounts as
`RP2350` mass storage. Copy the UF2:

```bash
cp build/dspico.uf2 /path/to/RP2350/
```

Debug prints come out on **UART0 (GPIO0 TX / GPIO1 RX, 115200 baud)** — USB is
reserved for the audio path.

---

## Running the go/no-go gates (human-in-the-loop)

**Phase 0 sanity** — on power-up the status LED (WS2812 on GPIO16) shows:
white (boot) → then amber (searching for DAC) / blue (enumerated by PC) /
green (streaming to DAC).

**Phase 1a — device** (Type-C1 → PC):
- The OS lists **"DSPico EQ Bridge"** as a selectable **output** device.
- It is **48 kHz / 24-bit stereo**, with a working **volume slider + mute**, and
  **no** microphone/recording device appears.
- Audio played to it is accepted (and silently discarded at this phase).

**Phase 1b — host** (Type-C2 → DAC) — *the real test*:
- Plug in one known class-compliant USB DAC. The firmware sets it to 48 kHz,
  opens its iso OUT endpoint, and streams a 1 kHz tone. LED goes **green**.
- **GATE:** a clean, continuous 1 kHz tone from the DAC for several minutes,
  verified **by ear and on a logic analyzer / USB trace**.

🚦 **If 1b cannot be stabilised, stop.** Switch platforms per brief §10
(ESP32-P4 with a real HS USB host, or RPi + CamillaDSP) rather than pushing on.

---

## Project layout

```
CMakeLists.txt          top-level build (SDK + Pico-PIO-USB + pioasm for LED)
pico_sdk_import.cmake    standard SDK locator
setup.sh                 clones lib/Pico-PIO-USB
docs/hardware-fix.md     board solder mods + meter-verify + decision flow
src/
  board_config.h         pins, 120 MHz clock, locked audio format
  tusb_config.h          TinyUSB config for BOTH device + host stacks
  usb_descriptors.[ch]   UAC2 speaker descriptors (Feature Unit, feedback EP)
  uac2_device.c          device audio callbacks: vol/mute, drain, feedback
  uac_host.[ch]          custom UAC *host* driver over Pico-PIO-USB (Phase 1b)
  test_tone.[ch]         48 kHz / 24-bit sine generator
  status_led.[ch]        WS2812 state indicator (on PIO2, no conflict)
  ws2812.pio             LED PIO program (assembled at build time)
  main.c                 clock, dual-core split, both stacks
```

## Next steps (after the gate passes)

Phase 2 wires the two stacks together: forward device-side received audio (the
`tud_audio_rx_done_post_read_cb` hook) into a capture ring, drain it from a play
ring in the host's `uac_host_fill_cb_t` source, replacing the test tone. Then
Phase 3 inserts pre-gain → biquad PEQ → host-volume and adds the WebUSB config
interface.
