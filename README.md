# DSPico

Inline digital **parametric-EQ USB bridge** for the Waveshare **RP2350-USB-C**
(default) / **RP2350-USB-CM** (`-DDSPICO_BOARD_USB_CM=ON`).

```
PC / phone ──USB──► [ DSPico = RP2350 ] ──USB──► USB DAC
                      applies PEQ (digital)
```

The bridge appears to the PC/phone as a driverless **USB Audio Class 2 (UAC2)
output** (48 kHz / 24-bit stereo), and re-transmits the (eventually EQ'd) stream
as a **USB host** to a downstream USB DAC. See `docs`/the build brief for the
full design.

---

## ⚠️ Current status

The **on-device parametric EQ is implemented and unit-tested**, and the full
PC → EQ → DAC signal path is wired. What is *not yet proven* is the one thing the
brief flagged as the risk: streaming isochronous audio as a host over the
bit-banged PIO port. That must be validated on real hardware (the Phase 1b gate)
before end-to-end audio can be trusted — but it does **not** block the EQ, which
is pure on-core DSP and runs regardless.

| Area | Status | Where |
|------|--------|-------|
| Dual-stack skeleton: device (core0) + PIO host (core1), 120 MHz, PIO polarity | ✅ implemented | `src/main.c`, `src/board_config.h` |
| **UAC2 output** — 48 kHz/24-bit stereo, no capture, Volume + Mute Feature Unit | ✅ implemented | `src/usb_descriptors.*`, `src/uac2_device.c` |
| **On-device parametric EQ** — N-band stereo SVF, 1 Hz–20 kHz, ±12 dB, Q 0.1–10, pre-gain, host-volume | ✅ implemented + **unit-tested** | `src/dsp_peq.*` |
| **Signal path** — PC audio → pre-gain → PEQ → host volume → cross-core ring → DAC | ✅ implemented + **unit-tested** | `src/signal_path.*`, `src/audio_ring.h` |
| **WebUSB configurator** — browser app + device vendor protocol, AutoEQ import, flash-persisted presets | ✅ implemented (app **unit-tested**) | `web/`, `src/config_usb.c` |
| **DAC format negotiation** — picks the DAC's stereo 48 kHz Type-I PCM alt; 24-bit preferred, 16-bit fallback (truncated) | ✅ implemented | `src/uac_host.c` |
| **Iso OUT over PIO-USB** — carried patch teaches Pico-PIO-USB isochronous OUT (no-handshake per spec) | ✅ implemented, ⚠️ **needs the hardware gate** | `patches/`, `src/uac_host.c` |
| Clock-domain feedback tuning; hot-plug/suspend robustness | ⛔ not yet (Phase 4/5) | — |

> Honesty note: the **DSP and signal-path code is verified** — compiled with the
> native compiler and checked numerically (see *Verifying the DSP* below), and
> the **full firmware cross-compiles in CI** against Pico SDK **2.1.1**
> (`.github/workflows/ci.yml` uploads the `.uf2`). Stock Pico-PIO-USB does
> **not** implement isochronous OUT — its OUT path waits for a handshake that
> iso never sends — so `setup.sh` pins the library to a known commit and applies
> [`patches/0001-host-iso-out-no-handshake.patch`](patches/). The patched iso
> path is spec-correct but **must still be validated on real hardware** (the
> Phase 1b gate): desk analysis cannot prove the bit-banged timing holds.

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
   the Type-C2 **D+ net** (**GPIO12 on -C**, **GPIO13 on -CM**) and **3V3**,
   after confirming with a meter:

   | Board variant | 1.5 kΩ D+ pull-up to remove | Empty footprint |
   |---------------|-----------------------------|-----------------|
   | **RP2350-USB-C**  | **R13** | R10 (NC) |
   | RP2350-USB-CM     | **R10** | R13 (NC) |

2. Still flaky / misses hot-plug? Add **~15 kΩ pull-downs** on D+ → GND and
   D− → GND.
3. Use a **self-powered DAC** — there is no VBUS switch/limit on the PIO port.
   A **captive or Type-A-cabled** DAC also avoids Type-C CC negotiation issues.
4. Leave the 27 Ω series resistors and the CC pull-downs alone — they're correct.

⚠️ **The D+/D− GPIO assignment is SWAPPED between the two variants** (verified
on the -C schematic): **-C: D+ = GPIO12, D− = GPIO13**; **-CM: D+ = GPIO13,
D− = GPIO12**. The firmware selects the right pinout at build time via
`DSPICO_BOARD_USB_CM` in `src/board_config.h` (default **-C**; configure with
`-DDSPICO_BOARD_USB_CM=ON` for -CM). This is config, not a solder fix — but
flashing the wrong variant's build means the DAC will **never** enumerate, so
check this before suspecting the resistor mod.

---

## Requirements

| Tool | Minimum version | Purpose |
|------|-----------------|---------|
| **ARM GNU toolchain** (`arm-none-eabi-gcc`) | 10 (13.x tested) | cross-compiler for the Cortex-M33 |
| **CMake** | 3.13 | build generator |
| **Make** or **Ninja** | any | build backend |
| **Git** | any | fetch the SDK + Pico-PIO-USB |
| **Python** | 3.x | SDK helper scripts + `picotool` build |
| **Pico SDK** | **2.0.0** (2.1.1 recommended) | RP2350 support (bundles TinyUSB) |
| **Pico-PIO-USB** | pinned commit (see `setup.sh`) | the second (host) USB port — fetched **and patched** by `setup.sh` |
| `libusb-1.0` dev headers | any | needed to build `picotool` (which the SDK builds) |

> RP2350 support only exists in Pico SDK **≥ 2.0.0** — an older SDK will fail.
> We build the ARM (Cortex-M33) core, so only the **`arm-none-eabi`** toolchain
> is needed (no RISC-V toolchain required).

---

## Step 1 — Install the toolchain (pick your OS)

### Debian / Ubuntu
```bash
sudo apt-get update
sudo apt-get install -y \
    cmake git python3 build-essential ninja-build \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    libusb-1.0-0-dev
```
> `libnewlib-arm-none-eabi` and `libstdc++-arm-none-eabi-newlib` are easy to
> forget and cause `cannot find -lc` / missing-header errors without them.

### Fedora / RHEL
```bash
sudo dnf install -y \
    cmake git python3 ninja-build \
    arm-none-eabi-gcc-cs arm-none-eabi-newlib libusbx-devel
```

### Arch / Manjaro
```bash
sudo pacman -S --needed \
    cmake git python ninja \
    arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-binutils libusb
```

### macOS (Homebrew)
```bash
brew install cmake git python ninja libusb
brew install --cask gcc-arm-embedded        # provides arm-none-eabi-gcc
# (alternative: brew install arm-none-eabi-gcc)
```

### Windows
Two good options:
- **Easiest:** the official **[Pico setup for Windows](https://github.com/raspberrypi/pico-setup-windows)**
  installer — it bundles the ARM toolchain, CMake, Ninja, Python, and the SDK,
  and gives you a preconfigured "Pico Developer Command Prompt". After it runs,
  skip Step 2 (it installs the SDK for you) and continue at Step 3.
- **Recommended for this project:** **WSL2** (Ubuntu) and then follow the
  Debian/Ubuntu instructions above — the flashing/serial notes below assume a
  Unix-like shell.

### Verify
```bash
arm-none-eabi-gcc --version   # expect 10.x–13.x
cmake --version               # expect >= 3.13
```

---

## Step 2 — Install the Pico SDK

You need the SDK on disk and the `PICO_SDK_PATH` environment variable pointing at
it. Clone it **with submodules** (this is what pulls in TinyUSB):

```bash
git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git --recurse-submodules ~/pico-sdk
export PICO_SDK_PATH=~/pico-sdk
# make it permanent:
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc     # or ~/.zshrc on macOS
```

> **Alternative (no manual clone):** skip this step and add
> `-DPICO_SDK_FETCH_FROM_GIT=ON` to the CMake command in Step 4 — CMake will
> download the SDK for you. The explicit clone above is faster for repeat builds.

---

## Step 3 — Fetch this project's extra dependency

From the repository root:

```bash
./setup.sh
```

This clones **Pico-PIO-USB** into `lib/Pico-PIO-USB` (the only vendored
dependency; the SDK provides everything else), pins it to a known-good commit,
and applies this repo's patches from [`patches/`](patches/) — currently the
**isochronous-OUT host support** the audio path requires (stock Pico-PIO-USB
waits for a handshake that iso transfers never send). Re-running it is safe;
don't clone the library by hand or you'll miss the patch.

---

## Step 4 — Build

```bash
cmake -B build -DPICO_BOARD=pico2          # add -G Ninja if you installed Ninja
cmake --build build -j
# -> build/dspico.uf2   (plus dspico.elf / .bin / .map)
```

The first configure builds `picotool` and generates `ws2812.pio.h` from the PIO
source — both automatic. A clean rebuild is `rm -rf build` then re-run the two
commands.

> **Flash size:** the build sets `PICO_FLASH_SIZE_BYTES` to **2 MB** (W25Q16
> per the board schematic), overriding the generic `pico2` board's 4 MB
> assumption. This matters because the EQ preset is persisted in the **last**
> flash sector. If your board carries a different part, adjust the definition
> in `CMakeLists.txt`.
>
> **Board variant:** the default build targets the **RP2350-USB-C**. For the
> **RP2350-USB-CM** add `-DDSPICO_BOARD_USB_CM=ON` to the configure step — the
> two variants have their PIO-USB D+/D− pins swapped and the wrong build will
> not enumerate the DAC.

---

## Step 5 — Flash

**BOOTSEL drag-and-drop (simplest):** hold **BOOTSEL**, tap **RUN/reset**,
release BOOTSEL → the board mounts as a `RP2350` USB drive. Copy the UF2:

```bash
# Linux (path varies by distro/user):
cp build/dspico.uf2 /media/$USER/RP2350/
# macOS:
cp build/dspico.uf2 /Volumes/RP2350/
# Windows: drag build\dspico.uf2 onto the RP2350 drive in Explorer
```

The board reboots into the firmware automatically after the copy.

**picotool (no button dance on reflash):**
```bash
picotool load build/dspico.uf2 && picotool reboot
```
On Linux, `picotool` needs USB permissions — either run with `sudo` or install
udev rules (the `pico-sdk/src/rp2_common/../picotool` repo ships
`99-picotool.rules`).

---

## Step 6 — View debug output (UART)

`printf` output goes to **UART0: GPIO0 = TX, GPIO1 = RX, 115200 8N1** — **not**
USB (both USB ports are the audio path). You need a **3.3 V USB-to-UART adapter**
wired GND↔GND, adapter-RX ↔ board-GPIO0.

```bash
# Linux (adapter usually enumerates as /dev/ttyUSB0):
sudo screen /dev/ttyUSB0 115200        # or: minicom -D /dev/ttyUSB0 -b 115200
# macOS:
screen /dev/tty.usbserial-XXXX 115200
# Windows: use PuTTY -> Serial -> COMx @ 115200
```
(Quit `screen` with `Ctrl-A` then `k`.)

---

## Troubleshooting

- **`PICO_SDK_PATH ... not found` / "SDK location was not specified"** — export
  `PICO_SDK_PATH` (Step 2) or configure with `-DPICO_SDK_FETCH_FROM_GIT=ON`.
- **`Pico-PIO-USB not found at lib/Pico-PIO-USB`** — run `./setup.sh` (Step 3).
- **`cannot find -lc` / `stdio.h: No such file`** — missing newlib; install
  `libnewlib-arm-none-eabi` (Debian) / `arm-none-eabi-newlib` (Fedora/Arch).
- **CMake picks the wrong chip** — always pass `-DPICO_BOARD=pico2` (RP2350).
- **`picotool` build fails on `libusb.h`** — install `libusb-1.0-0-dev`
  (or `libusbx-devel` / `libusb`).
- **Board never appears as `RP2350` drive** — you didn't enter BOOTSEL: hold
  BOOTSEL *before* tapping reset, keep holding a moment after.
- **TinyUSB audio-macro compile errors** — you're likely on a different SDK than
  2.1.1; the UAC2 descriptor macro names are version-sensitive (see the status
  note at the top). Report the error and it can be adjusted.

---

## First-boot troubleshooting (read the LED first)

The status LED (WS2812 on GPIO16) tells you which half is stuck, host side
taking priority:

| Colour | Meaning |
|--------|---------|
| dim white | booting |
| **amber** | nothing usable on either port — no DAC detected on Type-C2 |
| **blue** | PC enumerated us; **no DAC seen** on the host port |
| **cyan** | DAC attached, negotiation/setup in progress (should be brief) |
| **green** | iso audio streaming to the DAC |
| **red** | DAC attached but **incompatible** (no stereo 48 kHz PCM alt) or setup failed |

**No sound to the DAC** — walk the LED:
- **Blue with the DAC plugged in** → the DAC never enumerates. Check: right
  board-variant build flashed? (-C vs -CM pins are swapped — see *Hardware
  mods*); self-powered DAC (the PIO port has no VBUS management); try the
  D+ pull-up removal per `docs/hardware-fix.md`; try a different cable
  (captive/Type-A-cabled DACs avoid Type-C CC issues).
- **Red** → the DAC enumerated but offers no stereo 48 kHz 16/24-bit PCM alt,
  or a setup step failed — the UART log (GPIO0, 115200) prints exactly which
  step (`SET_INTERFACE`, sample rate, endpoint open) and the DAC's VID:PID.
- **Cyan forever** → a setup control transfer is hanging; UART shows the last
  step reached.
- **Green but silent** → the stream is running; check the PC actually plays to
  "DSPico EQ Bridge" and the OS volume/mute, then suspect iso data timing (the
  Phase 1b analyzer check below).

**WebUSB configurator won't connect** (the page's log panel names the failing
step and prints hints):
- **Windows:** the WinUSB driver must bind to the config interface via the
  MS OS 2.0 descriptor, and Windows **caches** that per firmware version. This
  firmware bumps the device version so a replug re-reads it; if it still
  fails, open Device Manager → find the DSPico entry → *Uninstall device*
  (tick "delete driver") → replug.
- **Linux:** Chrome needs rw access to the USB node:
  `sudo cp docs/99-dspico.rules /etc/udev/rules.d/ && sudo udevadm control --reload`,
  then replug.
- Works only in **Chrome/Edge/Chromium** over **https or localhost**, and the
  device must not be held open by another tab or app.

---

## Running the go/no-go gates (human-in-the-loop)

**Phase 0 sanity** — on power-up the status LED (WS2812 on GPIO16) shows:
white (boot) → then amber/blue (no DAC) → cyan (DAC negotiating) →
green (streaming to DAC); red = incompatible DAC (see the table above).

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

## Runtime behavior (stability rules baked into the firmware)

- **DAC format negotiation** — on enumeration the host driver reads the DAC's
  descriptors and picks a **stereo 48 kHz Type-I PCM** alternate setting:
  **24-bit (3-byte subslot) preferred**, with a **16-bit fallback** (samples
  truncated to their top 16 bits). A DAC offering neither is rejected with a
  clear UART message instead of being fed garbage. The choice is logged on the
  debug UART (`DSPico host: DAC itf … alt …`).
- **Priming** — playback toward the DAC starts only after ~8 ms of audio is
  buffered (`SIGNAL_PATH_PRIME_BYTES`), so a stream opens with a cushion
  instead of stuttering on scheduling jitter; an underrun silently re-primes.
- **Underrun = silence, not tone** — while the PC is streaming, any gap is
  filled with silence. The 1 kHz test tone plays only when *no* PC stream is
  active (bench/gate mode).
- **Overflow drops whole frames** — if the PC sends while no DAC drains, the
  cross-core ring drops complete frames (newest first) and never shifts the
  producer/consumer frame alignment (regression-tested in `tests/path_test.c`).
- **Stream restarts flush stale audio** — (re)starting the PC stream discards
  whatever tail the previous stream left in the ring.
- **Click-free volume** — the applied host gain slews toward the OS volume
  target per sample (full scale in ~5 ms), so volume steps and mute never
  click; a config **reset snaps** the gain so it can't ramp through the wrong
  loudness. 0 dB bands are skipped outright (output-identical, cycles saved).
- **CRC-protected presets** — the flash blob carries a CRC-32 over its payload;
  a torn write (power loss mid-commit) is rejected at boot and the device
  starts flat instead of loading garbage coefficients. Commits are also
  deferred out of the USB control callback (ACK first, write from the main
  loop) so a commit can't stall enumeration.
- **FPU flush-to-zero** — both cores run with FZ+DN set, so decaying filter
  state can't drag the M33 FPU through denormal territory during silence and
  spike the DSP time.

---

## Project layout

```
CMakeLists.txt          top-level build (SDK + Pico-PIO-USB + pioasm for LED)
pico_sdk_import.cmake    standard SDK locator
setup.sh                 clones lib/Pico-PIO-USB at a pinned commit + applies patches/
patches/                 carried Pico-PIO-USB patches (iso-OUT host support)
docs/hardware-fix.md     board solder mods + meter-verify + decision flow
src/
  board_config.h         pins, 120 MHz clock, locked audio format
  tusb_config.h          TinyUSB config for BOTH device + host stacks
  usb_descriptors.[ch]   UAC2 speaker descriptors (Feature Unit, feedback EP)
  uac2_device.c          device audio callbacks: vol/mute, RX->signal path
  uac_host.[ch]          custom UAC *host* driver over Pico-PIO-USB (Phase 1b)
  dsp_peq.[ch]           on-device parametric EQ engine (state-variable)  <<<
  signal_path.[ch]       pre-gain -> PEQ -> host volume -> play ring    <<<
  audio_ring.h           lock-free cross-core SPSC audio ring
  config_usb.[ch]        WebUSB vendor protocol + flash-persisted presets <<<
  test_tone.[ch]         48 kHz / 24-bit sine generator (idle/gate fallback)
  status_led.[ch]        WS2812 state indicator (on PIO2, no conflict)
  ws2812.pio             LED PIO program (assembled at build time)
  main.c                 clock, dual-core split, both stacks, EQ wiring
web/
  index.html             the configurator page
  dspico.js              UI, AutoEQ import, response curve, WebUSB protocol
tests/
  peq_test.c             native tests for the EQ engine
  path_test.c            native tests for the signal path + cross-core ring
  web_test.js            Node tests for the configurator's pure logic
  run.sh                 builds + runs every host-side suite
```

---

## On-device parametric EQ

The EQ runs **on the bridge**, not the PC (that's the whole point — it works from
a phone/tablet with no software). `src/dsp_peq.c` is a stereo chain of up to
`PEQ_MAX_BANDS` (16) state-variable filters in float32, with a global **pre-gain**
(headroom) stage in front and the **host volume** applied at the end:

```
PC audio → PRE-GAIN → biquad[0..N) → HOST VOLUME → DAC
```

Band types: **peaking**, **low-shelf**, **high-shelf**, **low-pass**,
**high-pass** — each with frequency, gain (dB), and Q. The device computes the
coefficients (it is the source of truth); a future WebUSB app only sends the
high-level band parameters.

**Parameter ranges** (enforced by `peq_set_band` — out-of-range values are
clamped; constants live in `dsp_peq.h`):

| Parameter | Range |
|-----------|-------|
| Frequency (Fc) | **1 Hz – 20 kHz** |
| Gain (peaking / shelf) | **−12 … +12 dB** |
| Q | **0.1 … 10** |

Each band is a **TPT state-variable filter** (Andrew Simper / Cytomic), not a
Direct-Form biquad. This matters for the low end: a float32 DF biquad collapses
near 1 Hz (its `cos(w0)` rounds to 1.0), whereas the SVF stays accurate — a
+6 dB band at 1 Hz measures **+6.00 dB**, and full ±12 dB at Q 0.1–10 is exact
across 1 Hz–20 kHz. Same per-sample cost.

**Setting a filter today** (until the WebUSB configurator lands, edit `main.c`):

```c
signal_path_set_pre_gain_db(-6.0f);                 // headroom for the boost
peq_band_t b = { .enabled = true, .type = PEQ_PEAKING,
                 .fc = 1000.0f, .gain_db = -6.0f, .q = 1.0f };
signal_path_set_band(0, &b);                         // band slot 0
```

A commented copy of exactly this (the brief's acceptance-test filter) sits in
`main.c` ready to uncomment. Volume/mute from the OS is applied automatically.

### Verifying the DSP (no hardware needed)

The EQ and signal-path modules are plain C and are validated with the native
compiler. The test drivers live in [`tests/`](tests/) and run in CI on every
push (see `.github/workflows/ci.yml`). To reproduce locally:

```bash
tests/run.sh          # builds + runs every host-side suite (needs gcc, node)
```

or individually:

```bash
# frequency response of designed filters (peaking/shelf/pre-gain/multi-band):
gcc -O2 -Wall -Wextra -I src -o peq_test tests/peq_test.c src/dsp_peq.c -lm && ./peq_test
# capture -> EQ -> ring -> play, incl. odd chunk sizes + wraparound:
gcc -O2 -Wall -Wextra -I src -o path_test tests/path_test.c src/signal_path.c src/dsp_peq.c -lm && ./path_test
```

Measured results: peaking/shelf gains land within ~0.15 dB of target at Fc and
are flat elsewhere; **the full range is exact — a +6 dB band reads +6.00 dB at
1 Hz and at 20 kHz, and ±12 dB holds across Q 0.1–10**; out-of-range params clamp
correctly; pre-gain scales exactly; a flat EQ is **bit-transparent — 0 LSB
deviation on the packed 24-bit path**; and 640k frames survive ring wraparound
with zero ordering errors. See [`tests/README.md`](tests/README.md) for the
full coverage map.

---

## Browser configurator (`web/`)

A single static page (`web/index.html` + `web/dspico.js`) that edits the EQ from
Chrome / Edge / Chromium. It works **standalone** (design a curve, import an
AutoEQ preset, see the response, save/load JSON) and **connected** (push bands +
pre-gain live over WebUSB, load the device's state, commit to flash).

**Hosted:** pushed to GitHub Pages at **https://lhj1086.github.io/DSPico/**
(auto-deployed from `web/` by `.github/workflows/pages.yml`). This is the URL the
device advertises as its WebUSB landing page.

**Or serve locally** (WebUSB needs https or localhost):

```bash
cd web && python3 -m http.server 8000
# open http://localhost:8000/ in Chrome/Edge, click "Connect device"
```

**AutoEQ compatibility.** Import a downloaded AutoEQ `ParametricEQ.txt` (file or
paste). It maps `Preamp` → pre-gain and the filter types **PK → peaking,
LSC/LS → low shelf, HSC/HS → high shelf, LP/LPQ → low pass, HP/HPQ → high pass**;
`ON`/`OFF` is honored; values outside DSPico's ranges (Fc 1 Hz–20 kHz, gain
±12 dB, Q 0.1–10) are clamped with a note, presets longer than the band count are
truncated with a note, and unsupported filter types (notch/allpass/bandpass) are
skipped with a note. The device uses the same band count it reports over USB.

**Device protocol.** Vendor control transfers on interface `ITF_NUM_VENDOR`
(`src/config_usb.c`), kept in sync with `web/dspico.js`: `INFO`, `GET_STATE`,
`SET_PREGAIN`, `SET_BAND`, `COMMIT` (persist to the last flash sector — ACKed
immediately, then written from the main loop with core1 briefly frozen), `RESET`. Driverless access uses a WebUSB BOS + MS OS
2.0 (WinUSB) descriptor. Update the landing-page URL in `usb_descriptors.c`
(`desc_url`) to wherever you host the page.

The app's pure logic (AutoEQ parser, clamping, response curve) is unit-tested
with Node in [`tests/web_test.js`](tests/web_test.js) (run via `tests/run.sh`);
the device half compiles with the rest of the USB stack.

## Next steps (after the gate passes)

- **Phase 4 — clock sync:** the capture (PC) and play (DAC) rates differ slightly;
  the async feedback endpoint already slaves the PC to us, but tune it against
  the real DAC FIFO level so the ring neither starves nor overflows over hours.
- **Phase 5 — robustness:** DAC hot-plug, PC suspend/resume, richer LED states,
  multiple stored presets.

---

## License

Released under the [MIT License](LICENSE).
