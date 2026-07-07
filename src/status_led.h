// ---------------------------------------------------------------------------
// DSPico — WS2812 status indicator (brief §2, §8 Phase 5 groundwork)
//
// Coarse, glanceable device state on the single onboard pixel:
//   BOOT       dim white  — powered, stacks not up yet
//   IDLE       dim blue   — PC enumerated us, no DAC on the host port
//   SEARCH     amber      — nothing on either port / waiting for the DAC
//   DAC_SETUP  cyan       — DAC attached, negotiation/setup in progress
//   STREAM     green      — iso audio flowing to the DAC
//   ERROR      red        — DAC attached but incompatible, or setup failed
// ---------------------------------------------------------------------------
#ifndef DSPICO_STATUS_LED_H
#define DSPICO_STATUS_LED_H

typedef enum {
  LED_BOOT = 0,
  LED_IDLE,
  LED_SEARCH,
  LED_DAC_SETUP,
  LED_STREAM,
  LED_ERROR,
} led_state_t;

// Initialise the WS2812 on its own PIO instance. Safe to call once at boot.
void status_led_init(void);

// Set the shown state (idempotent; cheap to call every loop iteration).
void status_led_set(led_state_t state);

#endif // DSPICO_STATUS_LED_H
