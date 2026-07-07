// ---------------------------------------------------------------------------
// DSPico — firmware-generated test tone (brief §8, Phase 1b)
//
// Produces a continuous stereo sine in the locked 48 kHz / 24-bit packed format
// so Phase 1b can stream a known signal to the DAC and verify it by ear + on a
// logic analyzer, with no PC audio involved yet.
// ---------------------------------------------------------------------------
#ifndef DSPICO_TEST_TONE_H
#define DSPICO_TEST_TONE_H

#include <stddef.h>
#include <stdint.h>

// Set the tone frequency (Hz) and level (dBFS, <= 0). Safe to call before use.
void test_tone_config(float freq_hz, float level_dbfs);

// Fill `dst` with `frames` stereo frames of 24-bit packed little-endian audio
// (6 bytes per frame: L[3] then R[3]). Phase advances across calls so
// consecutive packets are seamless. Returns frames written (== `frames`), the
// same unit signal_path_pull_play() returns, so callers can't mix the two up.
size_t test_tone_fill(uint8_t *dst, size_t frames);

#endif // DSPICO_TEST_TONE_H
