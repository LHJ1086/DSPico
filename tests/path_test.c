// ---------------------------------------------------------------------------
// DSPico — native tests for the audio signal path + cross-core ring.
//
// Properties verified (no hardware needed):
//   1. The lock-free SPSC ring preserves byte order across many wraparounds.
//   2. Feeding the signal path in odd, frame-unaligned chunks produces exactly
//      the same output as processing the whole buffer at once (the
//      carry/partial-frame logic is transparent).
//   3. Overflowing the play ring drops WHOLE frames only — the 6-byte frame
//      alignment between producer and consumer survives, and later audio is
//      still byte-exact (regression for the byte-truncated-write bug).
//   4. A stream (re)start flushes the previous stream's stale tail from the
//      ring (the epoch mechanism), and playback is held back until the
//      priming cushion (SIGNAL_PATH_PRIME_BYTES) has accumulated.
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

// The consumer (pull side) tracks the stream epoch lazily; one empty pull
// after configuration syncs it the way core1's continuous polling would.
static void sync_consumer(void) {
  uint8_t scrap[FRAME_BYTES];
  (void) signal_path_pull_play(scrap, 1);
}

// Flat-process `frames` frames of `src` into `dst` with an identically
// configured standalone PEQ — the byte-exact reference for the signal path.
static void make_reference(uint8_t *dst, const uint8_t *src, size_t frames,
                           float pre_gain_db, const peq_band_t *band) {
  peq_t ref;
  peq_init(&ref, (float) DSPICO_SAMPLE_RATE_HZ);
  if (pre_gain_db != 0.0f) peq_set_pre_gain_db(&ref, pre_gain_db);
  if (band) peq_set_band(&ref, 0, band);
  memcpy(dst, src, frames * FRAME_BYTES);
  peq_process_interleaved_s24(&ref, dst, frames);
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
  make_reference(ref, in, (size_t) FRAMES, -3.0f, &band);

  // Signal path: same config, fed in random odd byte chunks and drained
  // incrementally so the finite play ring never overflows (no dropped frames).
  signal_path_init();
  signal_path_set_pre_gain_db(-3.0f);
  signal_path_set_band(0, &band);
  signal_path_on_stream_start();
  sync_consumer();                     // adopt the new epoch before pushing

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

  // Priming may strand a sub-cushion tail once the source stops; everything
  // that did come out must be the exact reference prefix.
  check(NBYTES - pulled <= (long) SIGNAL_PATH_PRIME_BYTES,
        "at most one priming cushion is left stranded when the source stops");
  check(pulled % FRAME_BYTES == 0, "output is whole frames");
  check(memcmp(out, ref, (size_t) pulled) == 0,
        "chunked path output matches whole-buffer reference byte-for-byte");

done:
  free(in); free(ref); free(out);
}

static void test_overflow_keeps_frame_alignment(void) {
  printf("signal path: ring overflow drops whole frames, alignment survives:\n");

  const long PUSH_FRAMES = 3000;                       // ~2.2x ring capacity
  const long CAP_FRAMES  = (AUDIO_RING_CAP - 1) / FRAME_BYTES;   // 1365
  uint8_t *in  = malloc((size_t) (PUSH_FRAMES * FRAME_BYTES));
  uint8_t *ref = malloc((size_t) (PUSH_FRAMES * FRAME_BYTES));
  uint8_t *out = malloc((size_t) (PUSH_FRAMES * FRAME_BYTES));
  if (!in || !ref || !out) { check(false, "test buffers allocated"); goto done; }

  srand(7);
  for (long i = 0; i < PUSH_FRAMES * FRAME_BYTES; i++) in[i] = (uint8_t) rand();

  signal_path_init();                                  // flat EQ (stateless)
  sync_consumer();
  make_reference(ref, in, (size_t) PUSH_FRAMES, 0.0f, NULL);

  // Overflow the ring in one burst: the PC keeps sending while no DAC drains.
  signal_path_push_capture(in, (uint32_t) (PUSH_FRAMES * FRAME_BYTES));

  long pulled = 0;
  uint8_t drain[4092];
  for (;;) {
    size_t got = signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES);
    if (!got) break;
    memcpy(out + pulled, drain, got * FRAME_BYTES);
    pulled += (long) got * FRAME_BYTES;
  }

  check(pulled == CAP_FRAMES * FRAME_BYTES,
        "survivors are exactly the whole frames that fit the ring");
  check(memcmp(out, ref, (size_t) pulled) == 0,
        "surviving frames are byte-exact (no partial-frame shift)");

  // The path must still be perfectly usable after the overflow.
  const long AFTER = 400;                              // > priming cushion
  signal_path_push_capture(in, (uint32_t) (AFTER * FRAME_BYTES));
  pulled = 0;
  for (;;) {
    size_t got = signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES);
    if (!got) break;
    memcpy(out + pulled, drain, got * FRAME_BYTES);
    pulled += (long) got * FRAME_BYTES;
  }
  check(pulled == AFTER * FRAME_BYTES && memcmp(out, ref, (size_t) pulled) == 0,
        "audio after the overflow is still byte-exact (alignment held)");

done:
  free(in); free(ref); free(out);
}

static void test_stream_restart_flush_and_priming(void) {
  printf("signal path: stream restart flushes stale tail; priming holds back:\n");

  const long BATCH = 600;                              // 3600 B > priming cushion
  uint8_t *in  = malloc((size_t) (BATCH * FRAME_BYTES));
  uint8_t *ref = malloc((size_t) (BATCH * FRAME_BYTES));
  uint8_t *out = malloc((size_t) ((BATCH + 16) * FRAME_BYTES));   // batch + filler frames
  if (!in || !ref || !out) { check(false, "test buffers allocated"); goto done; }

  srand(42);
  for (long i = 0; i < BATCH * FRAME_BYTES; i++) in[i] = (uint8_t) rand();

  signal_path_init();                                  // flat EQ
  sync_consumer();

  // Stale tail from a previous stream sits in the ring...
  uint8_t stale[100 * FRAME_BYTES];
  memset(stale, 0x55, sizeof(stale));
  signal_path_push_capture(stale, sizeof(stale));

  // ...then a new stream starts.
  signal_path_on_stream_start();
  signal_path_push_capture(in, (uint32_t) (BATCH * FRAME_BYTES));

  // The consumer's first pull after the epoch bump discards everything queued
  // so far (stale tail plus at worst the new stream's first packets).
  uint8_t drain[4092];
  size_t first = signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES);
  check(first == 0, "first pull after restart returns nothing (flush + reprime)");

  // Under the priming threshold nothing comes out yet...
  uint8_t small[10 * FRAME_BYTES];
  memset(small, 0xAA, sizeof(small));
  signal_path_push_capture(small, sizeof(small));      // 60 B << cushion
  check(signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES) == 0,
        "below the priming cushion the consumer stays silent");

  // ...and once enough is buffered, playback starts with exactly the frames
  // pushed after the flush (0xAA filler + fresh batch), byte-exact.
  signal_path_push_capture(in, (uint32_t) (BATCH * FRAME_BYTES));
  long pulled = 0;
  for (;;) {
    size_t got = signal_path_pull_play(drain, sizeof(drain) / FRAME_BYTES);
    if (!got) break;
    memcpy(out + pulled, drain, got * FRAME_BYTES);
    pulled += (long) got * FRAME_BYTES;
  }
  uint8_t small_ref[sizeof(small)];
  make_reference(small_ref, small, 10, 0.0f, NULL);
  make_reference(ref, in, (size_t) BATCH, 0.0f, NULL);
  check(pulled == (long) sizeof(small) + BATCH * FRAME_BYTES,
        "everything pushed after the flush is delivered");
  check(pulled >= (long) sizeof(small) &&
        memcmp(out, small_ref, sizeof(small)) == 0 &&
        memcmp(out + sizeof(small), ref, (size_t) (pulled - (long) sizeof(small))) == 0,
        "no stale pre-restart bytes leak into the new stream");

done:
  free(in); free(ref); free(out);
}

int main(void) {
  printf("=== DSPico signal-path & ring tests ===\n");
  test_ring_wraparound();
  test_signalpath_chunking();
  test_overflow_keeps_frame_alignment();
  test_stream_restart_flush_and_priming();
  printf("\n%d/%d checks passed.\n", g_total - g_fail, g_total);
  if (g_fail) { printf("FAILED (%d check(s))\n", g_fail); return 1; }
  printf("OK\n");
  return 0;
}
