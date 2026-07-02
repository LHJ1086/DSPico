# Tests

Host-side unit tests for the parts of DSPico that are pure logic — the DSP
engine, the signal path, the cross-core ring, and the browser configurator's
math. **None of these need the Pico SDK, the toolchain, or hardware**: they
compile and run on any desktop with `gcc` (and `node` for the web half), which
is why they can gate every push in CI.

Run everything:

```bash
tests/run.sh
```

or individually:

```bash
# EQ engine: sine-driven magnitude response, exactness, clamping
gcc -O2 -Wall -Wextra -I src -o peq_test tests/peq_test.c src/dsp_peq.c -lm && ./peq_test

# Signal path + ring: odd-chunk framing, 640k-frame wraparound ordering
gcc -O2 -Wall -Wextra -I src -o path_test tests/path_test.c \
    src/signal_path.c src/dsp_peq.c -lm && ./path_test

# Configurator logic: AutoEQ import, clamping, response curve
node tests/web_test.js
```

Each program prints a check summary and exits non-zero on failure.

## What is covered

| Suite | Verifies |
|-------|----------|
| `peq_test.c`  | Flat EQ is transparent (float) / within 1 LSB (24-bit); pre-gain and host volume scale exactly; out-of-range Fc/gain/Q clamp; suggested pre-gain tracks the largest boost; peaking gain lands on target at Fc and is flat away from it; **±12 dB holds across 1 Hz–20 kHz and Q 0.1–10** (a +6 dB band reads +6 dB at 1 Hz and 20 kHz); shelf plateaus reach target. |
| `path_test.c` | The SPSC ring preserves byte order across many wraparounds (640k frames); feeding the signal path in odd, frame-unaligned chunks yields byte-for-byte the same output as processing the whole buffer at once (carry/partial-frame logic is transparent). |
| `web_test.js` | AutoEQ import (types, ON/OFF, clamping+warnings, unsupported-type skip, over-length truncation, default Q); `clampBand`; the response-curve math sums pre-gain and band gain and peaks at Fc. |

## Not covered here

The USB-glue code (`uac2_device.c`, `uac_host.c`, `usb_descriptors.c`,
`config_usb.c`) depends on TinyUSB / Pico-PIO-USB and is exercised by the
firmware cross-build in CI and, ultimately, the on-hardware go/no-go gates in
the top-level README — not by these host tests.
