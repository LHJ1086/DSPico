// ---------------------------------------------------------------------------
// DSPico — native tests for the audio signal path + cross-core ring.
//
// Two properties matter here and neither needs hardware:
//   1. The lock-free SPSC ring preserves byte order across many wraparounds.
//   2. Feeding the signal path in odd, frame-unaligned chunks produces exactly
//      the same output as processing the whole buffer at once (the carry/partial
//      -frame logic must be transparent). Because the SVF is a per-sample
//      recurrence, chunking cannot change the result — so any mismatch is a bug.
//
// Build & run:
//   gcc -O2 -Wall -Wextra -I src -o path_test tests/path_test.c src/signal_path.c src/dsp_peq.c -lm
//   ./path_test
// (or just run tests/run.sh). Exits non-zero if any check fails.
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_ring.h"
#include "board_config.h"
#include "signal_path.h"
#include "dsp_peq.h"

#define FRAME_BYTES (DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE)   // 6

static int g_fail = 0, g_total = 0;
static void check(bool cond, const char *msg) {
  g_total++;
  if (!cond) { g_fail++; printf("  FAIL: %s\n", msg); }
}

static void test_ring_wraparound(void) {
  printf("SPSC ring preserves byte order across many wraparounds:\n");
  audio_ring_t ring; audio_ring_init(&ring);

  const long TOTAL = 640000L * FRAME_BYTES;   // 640k stereo frames' worth of bytes
  long written = 0, readc = 0;
  uint32_t wseq = 0, rseq = 0;                // producer/consumer byte counters
  uint8_t tmp[257];
  bool order_ok = true;
  srand(999);

  while (readc < TOTAL) {
    // Produce an odd-sized chunk (whatever the ring will accept).
    if (written < TOTAL) {
      uint32_t want = (uint32_t) (rand() % 257);
      if ((long) want > TOTAL - written) want = (uint32_t) (TOTAL - written);
      for (uint32_t i = 0; i < want; i++) tmp[i] = (uint8_t) (wseq++);
      uint32_t got = audio_ring_write(&ring, tmp, want);
      written += got;
      wseq -= (want - got);                   // rewind seq for bytes not taken
    }
    // Consume an odd-sized chunk and verify the running counter.
    uint32_t rgot = audio_ring_read(&ring, tmp, (uint32_t) (rand() % 257));
    for (uint32_t i = 0; i < rgot; i++)
      if (tmp[i] != (uint8_t) (rseq++)) order_ok = false;
    readc += rgot;
  }

  check(order_ok, "640k frames survive wraparound with zero ordering errors");
  check(written == TOTAL && readc == TOTAL, "every byte written was read back");
}

static void test_signalpath_chunking(void) {
  printf("signal path: odd input chunks == whole-buffer processing:\n");

  const long FRAMES = 640000L;
  const long NBYTES = FRAMES * FRAME_BYTES;
  uint8_t *in  = malloc((size_t) NBYTES);
  uint8_t *ref = malloc((size_t) NBYTES);
  uint8_t *out = malloc((size_t) NBYTES);
  if (!in || !ref || !out) { check(false, "test buffers allocated"); goto done; }

  srand(2024);
  for (long i = 0; i < NBYTES; i++) in[i] = (uint8_t) rand();

  // A non-trivial EQ so the test exercises real coefficients, not a bypass.
  peq_band_t band = { .enabled = true, .type = PEQ_PEAKING, .fc = 2000, .gain_db = 5, .q = 1.2f };

  // Reference: one fresh, identically-configured PEQ over the whole buffer.
  peq_t ref_peq; peq_init(&ref_peq, (float) DSPICO_SAMPLE_RATE_HZ);
  peq_set_pre_gain_db(&ref_peq, -3.0f);
  peq_set_band(&ref_peq, 0, &band);
  memcpy(ref, in, (size_t) NBYTES);
  peq_process_interleaved_s24(&ref_peq, ref, (size_t) FRAMES);

  // Signal path: same config, fed in random odd byte chunks and drained
  // incrementally so the finite play ring never overflows (no dropped frames).
  signal_path_init();
  signal_path_set_pre_gain_db(-3.0f);
  signal_path_set_band(0, &band);
  signal_path_on_stream_start();

  long pushed = 0, pulled = 0;
  uint8_t drain[4092];                          // 682 frames, frame-aligned
  while (pushed < NBYTES) {
    uint32_t chunk = (uint32_t) (1 + rand() % 777);   // odd, frame-unaligned
    if ((long) chunk > NBYTES - pushed) chunk = (uint32_t) (NBYTES - pushed);
    signal_path_push_capture(in + pushed, chunk);
    pushed += chunk;
    for (;;) {
      size_t got = signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES);
      if (!got) break;
      memcpy(out + pulled, drain, got * FRAME_BYTES);
      pulled += (long) got * FRAME_BYTES;
    }
  }

  check(pulled == NBYTES, "all frames flushed through the path");
  check(pulled == NBYTES && memcmp(out, ref, (size_t) NBYTES) == 0,
        "chunked path output matches whole-buffer reference byte-for-byte");

done:
  free(in); free(ref); free(out);
}

int main(void) {
  printf("=== DSPico signal-path & ring tests ===\n");
  test_ring_wraparound();
  test_signalpath_chunking();
  printf("\n%d/%d checks passed.\n", g_total - g_fail, g_total);
  if (g_fail) { printf("FAILED (%d check(s))\n", g_fail); return 1; }
  printf("OK\n");
  return 0;
}
