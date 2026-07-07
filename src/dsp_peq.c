// ---------------------------------------------------------------------------
// DSPico — parametric EQ engine implementation.
//
// Each band is a TPT state-variable filter (Andrew Simper / Cytomic, "Solving
// the continuous SVF equations using trapezoidal integration"), chosen over
// Direct-Form RBJ biquads because it stays accurate in float32 down to ~1 Hz.
// ---------------------------------------------------------------------------
#include <math.h>
#include <string.h>

#include "dsp_peq.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define S24_NEG_FULL_SCALE  8388608.0f    //  2^23 (scale for both read + write)

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

// A band whose response is exactly unity (0 dB peaking/shelf) is skipped
// outright — the SVF would return the input bit-for-bit anyway (m0=1, m1=m2=0),
// so skipping is output-identical and saves the filter work.
static bool band_is_identity(const peq_band_t *b) {
  return (b->type == PEQ_PEAKING || b->type == PEQ_LOWSHELF ||
          b->type == PEQ_HIGHSHELF) &&
         fabsf(b->gain_db) < 0.01f;
}

void peq_init(peq_t *p, float fs) {
  memset(p, 0, sizeof(*p));
  p->fs            = fs;
  p->pre_gain      = 1.0f;
  p->host_gain     = 1.0f;
  p->host_gain_cur = 1.0f;
  // Slew the applied host gain across ~5 ms for a full 0..1 swing, so volume
  // steps and mute are click-free but still feel instant.
  p->gain_step     = 1.0f / (0.005f * fs);
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
  p->band_active[idx] = p->band[idx].enabled && !band_is_identity(&p->band[idx]);
}

void peq_set_pre_gain_db(peq_t *p, float db) {
  p->pre_gain = powf(10.0f, db / 20.0f);
}

void peq_set_host_gain(peq_t *p, float linear) {
  p->host_gain = linear;          // target; the frame loops slew toward it
}

void peq_set_host_gain_now(peq_t *p, float linear) {
  p->host_gain     = linear;
  p->host_gain_cur = linear;
}

void peq_recompute(peq_t *p) {
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    design_band(&p->band[i], p->fs, &p->coeffs[i]);
    p->band_active[i] = p->band[i].enabled && !band_is_identity(&p->band[i]);
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

// Run one channel's filter chain (pre-gain -> active bands). Host volume is
// applied by the frame loops so its slewed value is identical for L and R.
static inline float process_chain(peq_t *p, uint8_t ch, float x) {
  x *= p->pre_gain;
  for (uint8_t i = 0; i < PEQ_MAX_BANDS; i++) {
    if (p->band_active[i]) {
      x = svf_step(&p->coeffs[i], &p->state[ch][i], x);
    }
  }
  return x;
}

// Advance the applied host gain one frame toward its target (bounded linear
// slew — click-free volume/mute). Collapses to a single compare when settled.
static inline float advance_host_gain(peq_t *p) {
  float cur = p->host_gain_cur;
  const float tgt = p->host_gain;
  if (cur != tgt) {
    const float d = tgt - cur;
    if      (d >  p->gain_step) cur += p->gain_step;
    else if (d < -p->gain_step) cur -= p->gain_step;
    else                        cur  = tgt;
    p->host_gain_cur = cur;
  }
  return cur;
}

void peq_process_stereo_f32(peq_t *p, float *l, float *r, size_t frames) {
  for (size_t n = 0; n < frames; n++) {
    const float g = advance_host_gain(p);
    l[n] = process_chain(p, 0, l[n]) * g;
    r[n] = process_chain(p, 1, r[n]) * g;
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

// Both branches scale by 2^23 — the same factor the read path divides by — so
// a flat EQ round-trips every 24-bit value exactly (scaling +FS by 2^23-1
// instead used to lose 1 LSB). The clamp keeps +FS at the representable max.
static inline int32_t clamp_s24(float f) {
  if (f >= 0.0f) {
    int32_t v = (int32_t) lrintf(f * S24_NEG_FULL_SCALE);
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

    const float g = advance_host_gain(p);
    l = process_chain(p, 0, l) * g;
    r = process_chain(p, 1, r) * g;

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
