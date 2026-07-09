// ---------------------------------------------------------------------------
// DSPico — diagnostic log plumbing.
//
// The interesting bring-up events happen on core1 (the USB host), but blocking
// core1 on a 115200-baud UART inside enumeration callbacks disturbs exactly
// the timing being debugged — and most users have no UART adapter anyway.
//
// So: dlog() (CORE1-ONLY producer) drops formatted lines into a lock-free
// ring; core0's main loop drains it to the UART *and* into a second buffer
// that the WebUSB configurator can read over the vendor protocol
// (REQ_GET_LOG). Bring-up becomes debuggable from the browser alone.
// ---------------------------------------------------------------------------
#ifndef DSPICO_DEBUG_LOG_H
#define DSPICO_DEBUG_LOG_H

#include <stddef.h>
#include <stdint.h>

// printf-style; call ONLY from core1 (single-producer ring). Messages should
// end with '\n'. When the ring is momentarily full the tail of a message is
// dropped rather than blocking.
void dlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Same, but for CORE0 producers (device-stack events, boot banner). Never
// blocks: the UART mirror is queued and trickled out by dlog_task() only
// while the UART FIFO has room (a blocking printf stalls tud_task for
// milliseconds and drops audio — the old periodic-tick bug).
void dlog0(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// core0 main loop: forward pending log bytes to the UART and the USB buffer.
void dlog_task(void);

// core0 (vendor control handler): drain up to `maxlen` buffered log bytes for
// the configurator. Returns the number of bytes written to `dst`.
uint16_t dlog_usb_read(uint8_t *dst, uint16_t maxlen);

#endif // DSPICO_DEBUG_LOG_H
