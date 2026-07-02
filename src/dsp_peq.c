// ---------------------------------------------------------------------------
// DSPico — parametric EQ engine implementation (RBJ Audio EQ Cookbook).
// ---------------------------------------------------------------------------
#include <math.h>
#include <string.h>

#include "dsp_peq.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define S24_POS_FULL_SCALE  8388607.0f    //  2^23 - 1
#define S24_NEG_FULL_SCALE  8388608.0f    //  2^23

// Identity biquad: y = x.
static void coeffs_bypass(biquad_coeffs_t *c) {
  c->b0 = 1.0f; c->b1 = 0.0f; c->b2 = 0.0f; c->a1 = 0.0f; c->a2 = 0.0f;
}

// Compute a0-normalised coefficients for one band (RBJ cookbook).
static void design_band(const peq_band_t *b, float fs, biquad_coeffs_t *c) {
  if (!b->enabled || b->fc <= 0.0f || b->fc >= fs * 0.5f || b->q <= 0.0f) {
    coeffs_bypass(c);
    return;
  }

  const float A     = powf(10.0f, b->gain_db / 40.0f);   // amplitude (peaking/shelf)
  const float w0    = 2.0f * (float) M_PI * b->fc / fs;
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * b->q);

  float b0, b1, b2, a0, a1, a2;

  switch (b->type) {
    case PEQ_PEAKING:
      b0 = 1.0f + alpha * A;
      b1 = -2.0f * cosw0;
      b2 = 1.0f - alpha * A;
      a0 = 1.0f + alpha / A;
      a1 = -2.0f * cosw0;
      a2 = 1.0f - alpha / A;
      break;

    case PEQ_LOWSHELF: {
      const float ap1 = A + 1.0f, am1 = A - 1.0f;
      const float twoSqrtAalpha = 2.0f * sqrtf(A) * alpha;
      b0 =        A * (ap1 - am1 * cosw0 + twoSqrtAalpha);
      b1 =  2.0f * A * (am1 - ap1 * cosw0);
      b2 =        A * (ap1 - am1 * cosw0 - twoSqrtAalpha);
      a0 =            (ap1 + am1 * cosw0 + twoSqrtAalpha);
      a1 = -2.0f *    (am1 + ap1 * cosw0);
      a2 =            (ap1 + am1 * cosw0 - twoSqrtAalpha);
      break;
    }

    case PEQ_HIGHSHELF: {
      const float ap1 = A + 1.0f, am1 = A - 1.0f;
      const float twoSqrtAalpha = 2.0f * sqrtf(A) * alpha;
      b0 =        A * (ap1 + am1 * cosw0 + twoSqrtAalpha);
      b1 = -2.0f * A * (am1 + ap1 * cosw0);
      b2 =        A * (ap1 + am1 * cosw0 - twoSqrtAalpha);
      a0 =            (ap1 - am1 * cosw0 + twoSqrtAalpha);
      a1 =  2.0f *    (am1 - ap1 * cosw0);
      a2 =            (ap1 - am1 * cosw0 - twoSqrtAalpha);
      break;
    }

    case PEQ_LOWPASS:
      b0 = (1.0f - cosw0) * 0.5f;
      b1 =  1.0f - cosw0;
      b2 = (1.0f - cosw0) * 0.5f;
      a0 =  1.0f + alpha;
      a1 = -2.0f * cosw0;
      a2 =  1.0f - alpha;
      break;

    case PEQ_HIGHPASS:
      b0 =  (1.0f + cosw0) * 0.5f;
      b1 = -(1.0f + cosw0);
      b2 =  (1.0f + cosw0) * 0.5f;
      a0 =   1.0f + alpha;
      a1 =  -2.0f * cosw0;
      a2 =   1.0f - alpha;
      break;

    default:
      coeffs_bypass(c);
      return;
  }

  const float inv_a0 = 1.0f / a0;
  c->b0 = b0 * inv_a0;
  c->b1 = b1 * inv_a0;
  c->b2 = b2 * inv_a0;
  c->a1 = a1 * inv_a0;
  c->a2 = a2 * inv_a0;
}

void peq_init(peq_t *p, float fs) {
  memset(p, 0, sizeof(*p));
  p->fs        = fs;
  p->pre_gain  = 1.0f;
  p->host_gain = 1.0f;
  p->n_bands   = PEQ_MAX_BANDS;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    p->band[i].enabled = false;
    p->band[i].type    = PEQ_PEAKING;
    p->band[i].fc      = 1000.0f;
    p->band[i].gain_db = 0.0f;
    p->band[i].q       = 1.0f;
    coeffs_bypass(&p->coeffs[i]);
  }
}

void peq_set_band(peq_t *p, uint8_t idx, const peq_band_t *band) {
  if (idx >= PEQ_MAX_BANDS) return;
  p->band[idx] = *band;
  design_band(&p->band[idx], p->fs, &p->coeffs[idx]);
}

void peq_set_pre_gain_db(peq_t *p, float db) {
  p->pre_gain = powf(10.0f, db / 20.0f);
}

void peq_set_host_gain(peq_t *p, float linear) {
  p->host_gain = linear;
}

void peq_recompute(peq_t *p) {
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    design_band(&p->band[i], p->fs, &p->coeffs[i]);
  }
}

void peq_reset_state(peq_t *p) {
  memset(p->state, 0, sizeof(p->state));
}

// One Direct-Form-I biquad step.
static inline float biquad_step(const biquad_coeffs_t *c, biquad_state_t *s, float x) {
  const float y = c->b0 * x + c->b1 * s->x1 + c->b2 * s->x2
                            - c->a1 * s->y1 - c->a2 * s->y2;
  s->x2 = s->x1; s->x1 = x;
  s->y2 = s->y1; s->y1 = y;
  return y;
}

// Run one channel's whole chain (pre-gain -> enabled biquads -> host volume).
static inline float process_sample(peq_t *p, uint8_t ch, float x) {
  x *= p->pre_gain;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    if (p->band[i].enabled) {
      x = biquad_step(&p->coeffs[i], &p->state[ch][i], x);
    }
  }
  return x * p->host_gain;
}

void peq_process_stereo_f32(peq_t *p, float *l, float *r, size_t frames) {
  for (size_t n = 0; n < frames; n++) {
    l[n] = process_sample(p, 0, l[n]);
    r[n] = process_sample(p, 1, r[n]);
  }
}

// --- 24-bit packed <-> float helpers ---------------------------------------
static inline int32_t rd_s24_le(const uint8_t *p) {
  int32_t v = (int32_t) (p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16));
  if (v & 0x00800000) v |= (int32_t) 0xFF000000;   // sign-extend
  return v;
}

static inline void wr_s24_le(uint8_t *p, int32_t v) {
  p[0] = (uint8_t) (v & 0xFF);
  p[1] = (uint8_t) ((v >> 8) & 0xFF);
  p[2] = (uint8_t) ((v >> 16) & 0xFF);
}

static inline int32_t clamp_s24(float f) {
  if (f >= 0.0f) {
    int32_t v = (int32_t) lrintf(f * S24_POS_FULL_SCALE);
    return v > 8388607 ? 8388607 : v;
  } else {
    int32_t v = (int32_t) lrintf(f * S24_NEG_FULL_SCALE);
    return v < -8388608 ? -8388608 : v;
  }
}

void peq_process_interleaved_s24(peq_t *p, uint8_t *buf, size_t frames) {
  for (size_t n = 0; n < frames; n++) {
    uint8_t *fl = buf + (n * 6) + 0;
    uint8_t *fr = buf + (n * 6) + 3;

    float l = (float) rd_s24_le(fl) / S24_NEG_FULL_SCALE;
    float r = (float) rd_s24_le(fr) / S24_NEG_FULL_SCALE;

    l = process_sample(p, 0, l);
    r = process_sample(p, 1, r);

    wr_s24_le(fl, clamp_s24(l));
    wr_s24_le(fr, clamp_s24(r));
  }
}

float peq_suggested_pre_gain_db(const peq_t *p) {
  float max_boost = 0.0f;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    if (p->band[i].enabled && p->band[i].gain_db > max_boost &&
        (p->band[i].type == PEQ_PEAKING || p->band[i].type == PEQ_LOWSHELF ||
         p->band[i].type == PEQ_HIGHSHELF)) {
      max_boost = p->band[i].gain_db;
    }
  }
  return -max_boost;
}
