// ---------------------------------------------------------------------------
// DSPico — WS2812 status indicator implementation.
//
// Uses PIO2 so it never contends with Pico-PIO-USB (which defaults to PIO0/PIO1).
// ---------------------------------------------------------------------------
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "board_config.h"
#include "status_led.h"
#include "ws2812.pio.h"   // generated from ws2812.pio by pico_generate_pio_header

#define LED_PIO   pio2
#define LED_FREQ  800000   // WS2812 800 kHz

static int  s_sm = -1;
static led_state_t s_state = LED_BOOT;
static bool s_ready = false;

static inline void put_pixel_grb(uint8_t r, uint8_t g, uint8_t b) {
  // WS2812 wants G, R, B in the top 24 bits (shifted left by 8).
  const uint32_t grb = ((uint32_t) g << 16) | ((uint32_t) r << 8) | (uint32_t) b;
  pio_sm_put_blocking(LED_PIO, s_sm, grb << 8u);
}

void status_led_init(void) {
  s_sm = pio_claim_unused_sm(LED_PIO, true);
  const uint offset = pio_add_program(LED_PIO, &ws2812_program);
  ws2812_program_init(LED_PIO, s_sm, offset, DSPICO_WS2812_PIN, LED_FREQ, false);
  s_ready = true;
  status_led_set(LED_BOOT);
}

void status_led_set(led_state_t state) {
  if (!s_ready) return;
  s_state = state;
  switch (state) {
    case LED_BOOT:   put_pixel_grb(4, 4, 4);   break;  // dim white
    case LED_IDLE:   put_pixel_grb(0, 0, 12);  break;  // dim blue
    case LED_SEARCH: put_pixel_grb(16, 8, 0);  break;  // amber
    case LED_STREAM: put_pixel_grb(0, 16, 0);  break;  // green
    case LED_ERROR:  put_pixel_grb(24, 0, 0);  break;  // red
  }
}
