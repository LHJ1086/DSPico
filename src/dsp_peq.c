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

// Identity SVF: out = v0 (m1 = m2 = 0).
static void coeffs_bypass(svf_coeffs_t *c) {
  c->a1 = 0.0f; c->a2 = 0.0f; c->a3 = 0.0f;
  c->m0 = 1.0f; c->m1 = 0.0f; c->m2 = 0.0f;
}

// Compute TPT-SVF coefficients for one band (Cytomic / Andrew Simper,
// "Solving the continuous SVF equations using trapezoidal integration").
static void design_band(const peq_band_t *b, float fs, svf_coeffs_t *c) {
  if (!b->enabled || b->fc <= 0.0f || b->fc >= fs * 0.5f || b->q <= 0.0f) {
    coeffs_bypass(c);
    return;
  }

  const float A     = powf(10.0f, b->gain_db / 40.0f);   // shelf/bell amplitude
  const float w     = (float) M_PI * b->fc / fs;         // prewarp argument
  float g = tanf(w);                                      // prewarped frequency
  float k;
  float m0, m1, m2;

  switch (b->type) {
    case PEQ_PEAKING:                                     // "bell"
      k  = 1.0f / (b->q * A);
      m0 = 1.0f; m1 = k * (A * A - 1.0f); m2 = 0.0f;
      break;

    case PEQ_LOWSHELF:
      g  = g / sqrtf(A);
      k  = 1.0f / b->q;
      m0 = 1.0f; m1 = k * (A - 1.0f); m2 = (A * A - 1.0f);
      break;

    case PEQ_HIGHSHELF:
      g  = g * sqrtf(A);
      k  = 1.0f / b->q;
      m0 = A * A; m1 = k * (1.0f - A) * A; m2 = (1.0f - A * A);
      break;

    case PEQ_LOWPASS:
      k  = 1.0f / b->q;
      m0 = 0.0f; m1 = 0.0f; m2 = 1.0f;
      break;

    case PEQ_HIGHPASS:
      k  = 1.0f / b->q;
      m0 = 1.0f; m1 = -k; m2 = -1.0f;
      break;

    default:
      coeffs_bypass(c);
      return;
  }

  const float a1 = 1.0f / (1.0f + g * (g + k));
  c->a1 = a1;
  c->a2 = g * a1;
  c->a3 = g * c->a2;
  c->m0 = m0;
  c->m1 = m1;
  c->m2 = m2;
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

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void peq_clamp_band(peq_band_t *band) {
  band->fc      = clampf(band->fc,      PEQ_FC_MIN_HZ,   PEQ_FC_MAX_HZ);
  band->gain_db = clampf(band->gain_db, PEQ_GAIN_MIN_DB, PEQ_GAIN_MAX_DB);
  band->q       = clampf(band->q,       PEQ_Q_MIN,       PEQ_Q_MAX);
}

void peq_set_band(peq_t *p, uint8_t idx, const peq_band_t *band) {
  if (idx >= PEQ_MAX_BANDS) return;
  p->band[idx] = *band;
  peq_clamp_band(&p->band[idx]);
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

// One TPT state-variable filter step (Cytomic).
static inline float svf_step(const svf_coeffs_t *c, svf_state_t *s, float v0) {
  const float v3 = v0 - s->ic2eq;
  const float v1 = c->a1 * s->ic1eq + c->a2 * v3;
  const float v2 = s->ic2eq + c->a2 * s->ic1eq + c->a3 * v3;
  s->ic1eq = 2.0f * v1 - s->ic1eq;
  s->ic2eq = 2.0f * v2 - s->ic2eq;
  return c->m0 * v0 + c->m1 * v1 + c->m2 * v2;
}

// Run one channel's whole chain (pre-gain -> enabled bands -> host volume).
static inline float process_sample(peq_t *p, uint8_t ch, float x) {
  x *= p->pre_gain;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    if (p->band[i].enabled) {
      x = svf_step(&p->coeffs[i], &p->state[ch][i], x);
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
