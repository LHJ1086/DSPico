// ---------------------------------------------------------------------------
// DSPico — test tone generator implementation.
// ---------------------------------------------------------------------------
#include <math.h>

#include "board_config.h"
#include "test_tone.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TONE_FULL_SCALE_24  8388607.0f   // 2^23 - 1

static float s_phase = 0.0f;          // radians
static float s_phase_inc = 0.0f;      // radians per sample
static float s_amplitude = 0.0f;      // 0..1 linear

void test_tone_config(float freq_hz, float level_dbfs) {
  s_phase_inc = 2.0f * (float) M_PI * freq_hz / (float) DSPICO_SAMPLE_RATE_HZ;
  if (level_dbfs > 0.0f) level_dbfs = 0.0f;
  s_amplitude = powf(10.0f, level_dbfs / 20.0f);
}

// Write a signed 24-bit sample as 3 little-endian bytes.
static inline void put_s24_le(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
}

size_t test_tone_fill(uint8_t *dst, size_t frames) {
  if (s_phase_inc == 0.0f) test_tone_config(1000.0f, -6.0f);  // sane default

  for (size_t i = 0; i < frames; i++) {
    const float s = sinf(s_phase) * s_amplitude;
    int32_t v = (int32_t) lrintf(s * TONE_FULL_SCALE_24);
    if (v > 8388607) v = 8388607;
    if (v < -8388608) v = -8388608;

    put_s24_le(dst + (i * 6) + 0, v);   // left
    put_s24_le(dst + (i * 6) + 3, v);   // right (same signal)

    s_phase += s_phase_inc;
    if (s_phase >= 2.0f * (float) M_PI) s_phase -= 2.0f * (float) M_PI;
  }
  return frames;   // frames, not bytes — same unit as signal_path_pull_play()
}
