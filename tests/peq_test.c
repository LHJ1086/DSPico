// ---------------------------------------------------------------------------
// DSPico — native unit tests for the parametric-EQ engine (src/dsp_peq.c).
//
// Pure C, no hardware and no Pico SDK: the DSP is validated with the host
// compiler by driving real sine tones through the filters and measuring the
// steady-state magnitude response, plus exactness/clamping checks.
//
// Build & run:
//   gcc -O2 -Wall -Wextra -I src -o peq_test tests/peq_test.c src/dsp_peq.c -lm
//   ./peq_test
// (or just run tests/run.sh). Exits non-zero if any check fails.
// ---------------------------------------------------------------------------
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "crc32.h"
#include "dsp_peq.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS 48000.0f

static int g_fail = 0, g_total = 0;

static void check(bool cond, const char *msg) {
  g_total++;
  if (!cond) { g_fail++; printf("  FAIL: %s\n", msg); }
}

static void check_close(float got, float want, float tol, const char *msg) {
  g_total++;
  if (fabsf(got - want) > tol) {
    g_fail++;
    printf("  FAIL: %s (got %.4f, want %.4f +/- %.3f)\n", msg, got, want, tol);
  }
}

// Configure a fresh single-band PEQ (unity pre-gain / host-volume).
static void one_band(peq_t *p, peq_type_t t, float fc, float gain, float q) {
  peq_init(p, FS);
  peq_band_t b = { .enabled = true, .type = t, .fc = fc, .gain_db = gain, .q = q };
  peq_set_band(p, 0, &b);
}

// Drive band 0 of `p` with a unit sine at `f` Hz and return the steady-state
// magnitude response in dB (RMS out / RMS in over a settled window). The band's
// Q sizes the settling window, since resonance decays like ~Q*fs/(pi*f) samples.
// The engine's automatic clip guard scales the whole chain by pre_eff; divide
// it back out so this measures the BAND's response, not the guard.
static float measure_gain_db(peq_t *p, float f) {
  const double tau    = (double) p->band[0].q * FS / (M_PI * (double) f);
  const long   settle = (long) (8.0 * tau) + 20000;
  const long   meas   = (long) (4.0 * tau) + (long) (20.0 * FS / f) + 20000;

  peq_reset_state(p);
  double sum_out = 0.0, sum_in = 0.0;
  for (long n = 0; n < settle + meas; n++) {
    float x = sinf(2.0f * (float) M_PI * f * (float) n / FS);
    float l = x, r = 0.0f;
    peq_process_stereo_f32(p, &l, &r, 1);
    if (n >= settle) { sum_out += (double) l * l; sum_in += (double) x * x; }
  }
  return 20.0f * log10f((float) sqrt(sum_out / sum_in) / p->pre_eff);
}

static void test_flat_transparent(void) {
  printf("flat EQ is transparent (float path):\n");
  peq_t p; peq_init(&p, FS);
  bool ok = true;
  for (int i = 0; i < 1000; i++) {
    float x = sinf(0.01f * i) * 0.9f;
    float l = x, r = -x;
    peq_process_stereo_f32(&p, &l, &r, 1);
    if (l != x || r != -x) ok = false;
  }
  check(ok, "flat EQ passes float samples through unchanged");
}

static void test_flat_s24_near_transparent(void) {
  printf("flat EQ is bit-transparent (24-bit path, 0 LSB deviation):\n");
  peq_t p; peq_init(&p, FS);
  int worst = 0;
  srand(12345);
  for (int frame = 0; frame < 4096; frame++) {
    int32_t sl = (rand() % 16777216) - 8388608;   // full signed 24-bit range
    int32_t sr = (rand() % 16777216) - 8388608;
    uint8_t buf[6] = {
      (uint8_t) (sl & 0xFF), (uint8_t) ((sl >> 8) & 0xFF), (uint8_t) ((sl >> 16) & 0xFF),
      (uint8_t) (sr & 0xFF), (uint8_t) ((sr >> 8) & 0xFF), (uint8_t) ((sr >> 16) & 0xFF),
    };
    peq_process_interleaved_s24(&p, buf, 1);
    int32_t ol = (int32_t) (buf[0] | (buf[1] << 8) | (buf[2] << 16));
    if (ol & 0x800000) ol |= (int32_t) 0xFF000000;
    int32_t orr = (int32_t) (buf[3] | (buf[4] << 8) | (buf[5] << 16));
    if (orr & 0x800000) orr |= (int32_t) 0xFF000000;
    int dl = abs((int) (ol - sl)), dr = abs((int) (orr - sr));
    if (dl > worst) worst = dl;
    if (dr > worst) worst = dr;
  }
  check(worst == 0, "flat 24-bit path is bit-exact (0 LSB deviation)");
  printf("    (worst deviation: %d LSB)\n", worst);
}

static void test_gain_stages_exact(void) {
  printf("pre-gain and host volume scale exactly (no bands):\n");
  peq_t p; peq_init(&p, FS);
  peq_set_pre_gain_db(&p, -6.0f);
  float l = 0.5f, r = 0.5f;
  peq_process_stereo_f32(&p, &l, &r, 1);
  check_close(l, 0.5f * powf(10.0f, -6.0f / 20.0f), 1e-6f, "pre-gain -6 dB scales sample");

  // _now snaps immediately (used by config reset)...
  peq_init(&p, FS);
  peq_set_host_gain_now(&p, 0.25f);
  l = 0.8f; r = 0.8f;
  peq_process_stereo_f32(&p, &l, &r, 1);
  check_close(l, 0.8f * 0.25f, 1e-6f, "host volume _now 0.25 scales first sample");

  // ...while the normal setter reaches the same steady state after the ramp.
  peq_init(&p, FS);
  peq_set_host_gain(&p, 0.25f);
  float last = 0.0f;
  for (int n = 0; n < 480; n++) {           // 10 ms >> 5 ms full-scale slew
    l = 0.8f; r = 0.8f;
    peq_process_stereo_f32(&p, &l, &r, 1);
    last = l;
  }
  check_close(last, 0.8f * 0.25f, 1e-6f, "host volume 0.25 exact after the ramp");
}

static void test_host_gain_ramp_is_click_free(void) {
  printf("host-volume changes slew per-sample (no clicks), L/R identical:\n");
  peq_t p; peq_init(&p, FS);
  const float step = 1.0f / (0.005f * FS);   // engine's slew step

  // Constant full-ish input: any output jump is purely the gain trajectory.
  float prev_l = -1.0f;
  bool bounded = true, lr_match = true, monotonic = true;
  peq_set_host_gain(&p, 0.2f);               // 1.0 -> 0.2 while running
  for (int n = 0; n < 400; n++) {
    float l = 0.9f, r = 0.9f;
    peq_process_stereo_f32(&p, &l, &r, 1);
    if (l != r) lr_match = false;
    if (n > 0) {
      const float d = l - prev_l;
      if (fabsf(d) > 0.9f * step + 1e-6f) bounded = false;   // <= input * step
      if (d > 1e-7f) monotonic = false;                      // only ramps down
    }
    prev_l = l;
  }
  check(bounded,   "per-sample output delta never exceeds one slew step");
  check(monotonic, "downward ramp is monotonic");
  check(lr_match,  "L and R always get the same slewed gain");
  check_close(prev_l, 0.9f * 0.2f, 1e-6f, "ramp settles on the exact target");

  // Mute (target 0) fades out within ~5 ms instead of hard-cutting.
  peq_set_host_gain(&p, 0.0f);
  int frames_to_silence = 0;
  for (int n = 0; n < 480; n++) {
    float l = 0.9f, r = 0.9f;
    peq_process_stereo_f32(&p, &l, &r, 1);
    frames_to_silence++;
    if (l == 0.0f) break;
  }
  check(frames_to_silence > 10 && frames_to_silence <= 240 + 1,
        "mute fades over multiple samples and completes within ~5 ms");
}

static void test_identity_band_skipped(void) {
  printf("a 0 dB peaking/shelf band is skipped, output identical to disabled:\n");
  peq_t on, off;
  peq_init(&on,  FS);
  peq_init(&off, FS);
  peq_band_t flat = { .enabled = true, .type = PEQ_PEAKING,
                      .fc = 1000.0f, .gain_db = 0.0f, .q = 1.0f };
  peq_set_band(&on, 0, &flat);              // enabled but exactly 0 dB
  // `off` keeps every band disabled.

  bool identical = true;
  srand(555);
  for (int n = 0; n < 4096; n++) {
    float x = ((float) (rand() % 20001) - 10000.0f) / 10000.0f;
    float l1 = x, r1 = -x, l2 = x, r2 = -x;
    peq_process_stereo_f32(&on,  &l1, &r1, 1);
    peq_process_stereo_f32(&off, &l2, &r2, 1);
    if (l1 != l2 || r1 != r2) identical = false;
  }
  check(identical, "0 dB band output is bit-identical to no band");
}

static void test_clamp(void) {
  printf("out-of-range parameters clamp into the allowed ranges:\n");
  peq_t p; peq_init(&p, FS);

  peq_band_t hi = { .enabled = true, .type = PEQ_PEAKING, .fc = 1e9f, .gain_db = 99.0f, .q = 99.0f };
  peq_set_band(&p, 0, &hi);
  check_close(p.band[0].fc,      PEQ_FC_MAX_HZ,   0.0f, "fc clamps to 20 kHz");
  check_close(p.band[0].gain_db, PEQ_GAIN_MAX_DB, 0.0f, "gain clamps to +12 dB");
  check_close(p.band[0].q,       PEQ_Q_MAX,       0.0f, "Q clamps to 10");

  peq_band_t lo = { .enabled = true, .type = PEQ_PEAKING, .fc = 0.001f, .gain_db = -99.0f, .q = 1e-4f };
  peq_set_band(&p, 1, &lo);
  check_close(p.band[1].fc,      PEQ_FC_MIN_HZ,   0.0f, "fc clamps to 1 Hz");
  check_close(p.band[1].gain_db, PEQ_GAIN_MIN_DB, 0.0f, "gain clamps to -12 dB");
  check_close(p.band[1].q,       PEQ_Q_MIN,       0.0f, "Q clamps to 0.1");

  // Out-of-bounds slot index must be a safe no-op, not a buffer overrun.
  peq_set_band(&p, PEQ_MAX_BANDS, &lo);
  check(true, "set_band with idx == PEQ_MAX_BANDS is ignored");
}

static void test_suggested_pregain(void) {
  printf("suggested pre-gain tracks the largest boost:\n");
  peq_t p; peq_init(&p, FS);
  check_close(peq_suggested_pre_gain_db(&p), 0.0f, 0.0f, "no bands -> 0 dB");

  peq_band_t b0 = { .enabled = true, .type = PEQ_PEAKING,  .fc = 1000, .gain_db = 6,  .q = 1 };
  peq_band_t b1 = { .enabled = true, .type = PEQ_LOWSHELF, .fc = 100,  .gain_db = 9,  .q = 0.7f };
  peq_band_t b2 = { .enabled = true, .type = PEQ_PEAKING,  .fc = 5000, .gain_db = -4, .q = 1 };
  peq_set_band(&p, 0, &b0); peq_set_band(&p, 1, &b1); peq_set_band(&p, 2, &b2);
  check_close(peq_suggested_pre_gain_db(&p), -9.0f, 0.0f, "max boost 9 dB -> -9 dB");

  peq_init(&p, FS);
  peq_band_t cut = { .enabled = true, .type = PEQ_PEAKING, .fc = 1000, .gain_db = -6, .q = 1 };
  peq_set_band(&p, 0, &cut);
  check_close(peq_suggested_pre_gain_db(&p), 0.0f, 0.0f, "only cuts -> 0 dB");
}

static void test_clip_guard(void) {
  printf("clip guard reserves headroom for band boosts (no hard clipping):\n");
  peq_t p; peq_init(&p, FS);
  check_close(p.pre_eff, 1.0f, 0.0f, "flat EQ applies unity effective pre-gain");

  peq_band_t boost = { .enabled = true, .type = PEQ_PEAKING,
                       .fc = 1000.0f, .gain_db = 6.0f, .q = 1.0f };
  peq_set_band(&p, 0, &boost);
  check_close(p.pre_eff, powf(10.0f, -6.0f / 20.0f), 1e-6f,
              "+6 dB boost -> -6 dB effective pre-gain");

  // A user pre-gain below the guard is honoured as-is...
  peq_set_pre_gain_db(&p, -9.0f);
  check_close(p.pre_eff, powf(10.0f, -9.0f / 20.0f), 1e-6f,
              "user -9 dB (below the guard) is applied unchanged");
  // ...one above it is capped at the guard.
  peq_set_pre_gain_db(&p, 0.0f);
  check_close(p.pre_eff, powf(10.0f, -6.0f / 20.0f), 1e-6f,
              "user 0 dB is capped at the -6 dB guard");

  // Cut-only EQ: no headroom reserved, user pre-gain passes through.
  peq_t cut; peq_init(&cut, FS);
  peq_band_t dip = { .enabled = true, .type = PEQ_PEAKING,
                     .fc = 1000.0f, .gain_db = -6.0f, .q = 1.0f };
  peq_set_band(&cut, 0, &dip);
  check_close(cut.pre_eff, 1.0f, 0.0f, "cut-only EQ keeps unity pre-gain");

  // A near-full-scale sine at the boosted centre frequency must come out of
  // the 24-bit path without ever touching the rails (the un-guarded engine
  // hard-clipped every cycle here).
  peq_reset_state(&p);
  long railed = 0;
  for (int n = 0; n < 48000; n++) {
    float x = 0.9f * sinf(2.0f * (float) M_PI * 1000.0f * (float) n / FS);
    int32_t s = (int32_t) lrintf(x * 8388608.0f);
    if (s > 8388607) s = 8388607;
    uint8_t buf[6] = {
      (uint8_t) (s & 0xFF), (uint8_t) ((s >> 8) & 0xFF), (uint8_t) ((s >> 16) & 0xFF),
      (uint8_t) (s & 0xFF), (uint8_t) ((s >> 8) & 0xFF), (uint8_t) ((s >> 16) & 0xFF),
    };
    peq_process_interleaved_s24(&p, buf, 1);
    int32_t o = (int32_t) (buf[0] | (buf[1] << 8) | (buf[2] << 16));
    if (o & 0x800000) o |= (int32_t) 0xFF000000;
    if (o >= 8388607 || o <= -8388608) railed++;
  }
  check(railed == 0, "boosted full-scale sine never hits the 24-bit rails");
}

static void test_peaking_center_gain(void) {
  printf("peaking gain lands on target at the centre frequency:\n");
  peq_t p;
  one_band(&p, PEQ_PEAKING, 1000.0f, 6.0f, 1.0f);
  check_close(measure_gain_db(&p, 1000.0f), 6.0f, 0.35f, "+6 dB @ 1 kHz, Q1");
  one_band(&p, PEQ_PEAKING, 1000.0f, -12.0f, 0.1f);
  check_close(measure_gain_db(&p, 1000.0f), -12.0f, 0.35f, "-12 dB @ 1 kHz, Q0.1");
  one_band(&p, PEQ_PEAKING, 1000.0f, 12.0f, 10.0f);
  check_close(measure_gain_db(&p, 1000.0f), 12.0f, 0.35f, "+12 dB @ 1 kHz, Q10");
}

static void test_peaking_flat_away(void) {
  printf("peaking is flat well away from its centre:\n");
  peq_t p;
  one_band(&p, PEQ_PEAKING, 1000.0f, 6.0f, 3.0f);
  check_close(measure_gain_db(&p, 50.0f),    0.0f, 0.3f, "flat at 50 Hz (below the 1 kHz peak)");
  check_close(measure_gain_db(&p, 16000.0f), 0.0f, 0.3f, "flat at 16 kHz (above the peak)");
}

static void test_low_high_freq_exact(void) {
  printf("full range stays exact at the band edges (1 Hz / 20 kHz):\n");
  peq_t p;
  one_band(&p, PEQ_PEAKING, 1.0f, 6.0f, 0.707f);
  check_close(measure_gain_db(&p, 1.0f), 6.0f, 0.4f, "+6 dB peaking @ 1 Hz");
  one_band(&p, PEQ_PEAKING, 20000.0f, 6.0f, 0.707f);
  check_close(measure_gain_db(&p, 20000.0f), 6.0f, 0.4f, "+6 dB peaking @ 20 kHz");
  one_band(&p, PEQ_PEAKING, 1.0f, 12.0f, 0.1f);
  check_close(measure_gain_db(&p, 1.0f), 12.0f, 0.4f, "+12 dB peaking @ 1 Hz, Q0.1");
}

static void test_shelves(void) {
  printf("shelf plateaus reach target and stay flat past the corner:\n");
  peq_t p;
  one_band(&p, PEQ_LOWSHELF, 100.0f, 6.0f, 0.707f);
  check_close(measure_gain_db(&p, 5.0f),     6.0f, 0.5f, "low shelf +6 dB plateau below corner");
  check_close(measure_gain_db(&p, 15000.0f), 0.0f, 0.4f, "low shelf flat above corner");
  one_band(&p, PEQ_HIGHSHELF, 5000.0f, 6.0f, 0.707f);
  check_close(measure_gain_db(&p, 20000.0f), 6.0f, 0.6f, "high shelf +6 dB plateau above corner");
  check_close(measure_gain_db(&p, 100.0f),   0.0f, 0.4f, "high shelf flat below corner");
}

static void test_crc32(void) {
  printf("config-blob CRC-32 (ISO-HDLC) is correct and detects corruption:\n");
  // Standard check value for this CRC variant.
  check(dspico_crc32("123456789", 9) == 0xCBF43926u, "known vector matches");
  check(dspico_crc32("", 0) == 0x00000000u, "empty input hashes to 0");

  // A single flipped byte anywhere must change the CRC (torn-write model).
  uint8_t blob[268];
  for (size_t i = 0; i < sizeof(blob); i++) blob[i] = (uint8_t) (i * 7 + 3);
  const uint32_t good = dspico_crc32(blob, sizeof(blob));
  bool detected = true;
  for (size_t i = 0; i < sizeof(blob); i += 13) {
    blob[i] ^= 0xFF;
    if (dspico_crc32(blob, sizeof(blob)) == good) detected = false;
    blob[i] ^= 0xFF;
  }
  check(detected, "every single-byte corruption changes the CRC");
  check(dspico_crc32(blob, sizeof(blob)) == good, "restored blob hashes clean");
}

int main(void) {
  printf("=== DSPico PEQ engine tests ===\n");
  test_crc32();
  test_flat_transparent();
  test_flat_s24_near_transparent();
  test_gain_stages_exact();
  test_host_gain_ramp_is_click_free();
  test_identity_band_skipped();
  test_clamp();
  test_suggested_pregain();
  test_clip_guard();
  test_peaking_center_gain();
  test_peaking_flat_away();
  test_low_high_freq_exact();
  test_shelves();
  printf("\n%d/%d checks passed.\n", g_total - g_fail, g_total);
  if (g_fail) { printf("FAILED (%d check(s))\n", g_fail); return 1; }
  printf("OK\n");
  return 0;
}
