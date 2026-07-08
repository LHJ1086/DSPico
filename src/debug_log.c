// ---------------------------------------------------------------------------
// DSPico — diagnostic log implementation (see debug_log.h for the design).
// ---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>

#include "pico.h"            // get_core_num()
#include "debug_log.h"

// --- core1 -> core0 ring (SPSC, acquire/release like audio_ring.h) ----------
#define XRING_CAP 2048u
static uint8_t  x_buf[XRING_CAP];
static uint32_t x_head;   // producer (core1)
static uint32_t x_tail;   // consumer (core0)

static void xring_write(const uint8_t *s, uint32_t len) {
  const uint32_t head = x_head;
  const uint32_t tail = __atomic_load_n(&x_tail, __ATOMIC_ACQUIRE);
  const uint32_t space = XRING_CAP - (head - tail) - 1;
  if (len > space) len = space;                 // drop the tail when full
  for (uint32_t i = 0; i < len; i++) {
    x_buf[(head + i) & (XRING_CAP - 1)] = s[i];
  }
  __atomic_store_n(&x_head, head + len, __ATOMIC_RELEASE);
}

void dlog(const char *fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int) sizeof line) n = (int) sizeof line;
  xring_write((const uint8_t *) line, (uint32_t) n);
}

// --- core0-only buffer feeding the WebUSB log read --------------------------
#define ULOG_CAP 8192u   // roomy so a burst of USB trace can't evict key lines
static uint8_t  u_buf[ULOG_CAP];
static uint32_t u_head, u_tail;

static void ulog_push(uint8_t c) {
  if (u_head - u_tail >= ULOG_CAP) u_tail++;    // full: drop the oldest byte
  u_buf[u_head++ & (ULOG_CAP - 1)] = c;
}

void dlog_task(void) {
  uint8_t tmp[64];                              // bounded work per loop pass
  const uint32_t tail = x_tail;
  const uint32_t head = __atomic_load_n(&x_head, __ATOMIC_ACQUIRE);
  uint32_t avail = head - tail;
  if (avail == 0) return;
  if (avail > sizeof tmp) avail = sizeof tmp;
  for (uint32_t i = 0; i < avail; i++) {
    tmp[i] = x_buf[(tail + i) & (XRING_CAP - 1)];
    ulog_push(tmp[i]);
  }
  __atomic_store_n(&x_tail, tail + avail, __ATOMIC_RELEASE);
  printf("%.*s", (int) avail, tmp);             // mirror to the UART
}

void dlog0(const char *fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int) sizeof line) n = (int) sizeof line;
  printf("%.*s", n, line);                      // UART immediately
  for (int i = 0; i < n; i++) ulog_push((uint8_t) line[i]);   // core0-only buffer
}

// TinyUSB internal-trace sink (wired via CFG_TUSB_DEBUG_PRINTF). The DEVICE
// stack runs on core0, so its trace goes straight to the WebUSB log buffer to
// expose SET_INTERFACE / endpoint-open behaviour during bring-up. HOST-stack
// trace (core1) is dropped before any formatting so it cannot add latency to
// the timing-critical PIO-USB path.
int dspico_tusb_printf(const char *fmt, ...) {
  if (get_core_num() != 0) return 0;            // drop host (core1) trace
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  if (n <= 0) return 0;
  if (n > (int) sizeof line) n = (int) sizeof line;
  printf("%.*s", n, line);                      // UART
  for (int i = 0; i < n; i++) ulog_push((uint8_t) line[i]);   // WebUSB log
  return n;
}

uint16_t dlog_usb_read(uint8_t *dst, uint16_t maxlen) {
  uint32_t avail = u_head - u_tail;
  if (avail > maxlen) avail = maxlen;
  for (uint32_t i = 0; i < avail; i++) {
    dst[i] = u_buf[(u_tail + i) & (ULOG_CAP - 1)];
  }
  u_tail += avail;
  return (uint16_t) avail;
}
