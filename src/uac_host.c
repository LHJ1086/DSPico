// ---------------------------------------------------------------------------
// DSPico — USB Audio host driver over Pico-PIO-USB (Phase 1b).
//
// Custom TinyUSB application host driver. See uac_host.h for the contract and
// the big caveat: isochronous OUT over the bit-banged PIO port is unproven, so
// treat this as the scaffold you iterate on with a logic analyzer.
// ---------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>

#include "pico/time.h"
#include "tusb.h"
#include "debug_log.h"
#include "host/usbh_pvt.h"   // usbh_class_driver_t, usbh_driver_set_config_complete
#include "board_config.h"
#include "signal_path.h"     // signal_path_play_fill() (heartbeat diagnostics)
#include "uac_host.h"

// Max audio payload we will ever put on the wire in one frame. The widest
// wire format is a 4-byte subslot (24-in-32), not the native 3-byte one.
#define UAC_HOST_PKT_MAX  ((DSPICO_SAMPLES_PER_MS + 1) * DSPICO_NUM_CHANNELS * 4u)

// Linux pacing quirk (QUIRK_FLAG_CTL_MSG_DELAY_5M): several USB-C dongle DACs
// (Samsung/AKG, Huawei, Vivo — all in sound/usb/quirks.c) misbehave when
// class-control requests arrive back-to-back; real hosts space them out. We
// pace EVERY setup-chain step by this much — costs ~50 ms once per attach,
// harmless for compliant DACs.
#define UAC_HOST_CTL_PACE_MS 5u

// ---------------------------------------------------------------------------
// Device state (single DAC supported for the spike)
// ---------------------------------------------------------------------------
typedef struct {
  bool     in_use;
  uint8_t  dev_addr;
  uint8_t  rhport;

  uint8_t  ac_itf;          // Audio Control interface number
  uint16_t audio_bcd;       // 0x0100 = UAC1, 0x0200 = UAC2

  // UAC2 clock topology. Headset codecs (e.g. the CX31988 family) expose more
  // than one Clock Source (DAC + ADC paths) and sometimes a Clock Selector; a
  // real host walks the chain, so we record every source and the selector and
  // program them all rather than betting on "the last one parsed".
  uint8_t  clock_ids[4];    // every UAC2 Clock Source entity seen
  uint8_t  clock_count;
  uint8_t  clock_sel_id;    // UAC2 Clock Selector entity (0 if none)

  // Feature Unit discovery, to unmute/set volume on the DAC's playback path
  // (real hosts always do this; DACs may power up muted or at minimum volume).
  // fu_ctrl[i][ch] holds the low byte of that FU's bmaControls for the channel
  // (bits 0-1 = Mute, bits 2-3 = Volume; 0b11 = host-writable). We send a
  // control ONLY where it exists: the CX31988 puts Mute on the master and
  // Volume on L/R only, and NAKs forever on a write to a control it lacks —
  // which used to wedge the very volume writes that unmute it, so it stayed
  // silent. FU_MAX_CH covers master + stereo.
#define FU_MAX_CH 3
  uint8_t  fu_ids[8];       // every Feature Unit seen in the AC block
  uint8_t  fu_ctrl[8][FU_MAX_CH];
  uint8_t  fu_nch[8];       // channels (incl. master) recorded for each FU
  uint8_t  fu_count;
  uint8_t  ot_source_id;    // bSourceID of the speaker/headphone Output Terminal
  uint8_t  fu_id;           // the FU we drive (resolved in dac_set_config)
  uint8_t  fu_ctrl_sel[FU_MAX_CH];   // control map of the resolved fu_id
  uint8_t  fu_ctrl_nch;              // its channel count

  uint8_t  as_itf;          // Audio Streaming interface number
  uint8_t  as_alt;          // operational alt setting (has the iso EP)
  bool     have_as;

  uint8_t  subslot;         // wire bytes/sample: 2, 3, or 4 (24-in-32)
  uint8_t  bits;            // valid bits/sample of the chosen alt (for diagnostics)
  uint8_t  cand_rank;       // ranking of the accepted candidate (lower = better)

  tusb_desc_endpoint_t ep_out;   // the isochronous OUT endpoint descriptor
  bool     have_ep;

  // Async-DAC clock tracking: the explicit feedback (iso IN) endpoint of the
  // chosen alt, if the data endpoint declares asynchronous sync.
  tusb_desc_endpoint_t ep_fb;
  bool     have_fb;
  bool     ep_async;        // data EP bmAttributes.sync == asynchronous

  bool     incompatible;    // audio device seen, but nothing we can stream to
  bool     streaming;
} dac_dev_t;

static dac_dev_t s_dac;
static uac_host_fill_cb_t s_fill_cb = NULL;

// Raw audio-function descriptor block, captured on attach and hex-dumped
// incrementally from uac_host_task() (one line per pass, so the diagnostic log
// ring never overflows). This is what finally reveals exactly what a stubborn
// DAC — e.g. the CX31988 — advertises, without needing a USB analyzer.
#define UAC_DESC_DUMP_MAX 512u
static uint8_t  s_desc_dump[UAC_DESC_DUMP_MAX];
static uint16_t s_desc_len;   // bytes captured
static uint16_t s_desc_pos;   // dump cursor (== s_desc_len when finished)

// One outstanding iso packet at a time; DMA-aligned.
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_pkt[UAC_HOST_PKT_MAX];

void uac_host_set_source(uac_host_fill_cb_t cb) { s_fill_cb = cb; }
bool uac_host_is_streaming(void) { return s_dac.in_use && s_dac.streaming; }

uac_host_state_t uac_host_state(void) {
  if (!s_dac.in_use)      return UAC_HOST_NO_DAC;
  if (s_dac.streaming)    return UAC_HOST_STREAMING;
  if (s_dac.incompatible || !s_dac.have_as) return UAC_HOST_INCOMPATIBLE;
  return UAC_HOST_SETUP;
}

// ---------------------------------------------------------------------------
// Descriptor parsing helpers
// ---------------------------------------------------------------------------
static uint8_t const *desc_next(uint8_t const *p) { return p + p[0]; }

// Format facts collected for one AS alternate setting while walking its
// descriptors; judged when we reach the alt's isochronous OUT data endpoint.
typedef struct {
  uint8_t itf, alt;
  uint8_t channels;    // 0 until seen
  uint8_t subslot;     // wire bytes per sample, 0 until seen
  uint8_t bits;        // valid bits per sample
  bool    pcm;         // format is Type I PCM
  bool    rate_ok;     // 48 kHz supported (assumed true unless a UAC1 list says no)
  bool    in_as_itf;   // currently inside an AS interface's descriptors
  // The alt's iso OUT data EP and (optional) iso IN feedback EP. Both are
  // collected while walking the alt; the candidate is judged when the alt
  // ends, because in descriptor order the feedback EP follows the data EP.
  tusb_desc_endpoint_t data_ep;
  bool    have_data;
  tusb_desc_endpoint_t fb_ep;
  bool    have_fb;
} alt_cand_t;

// Rank of a wire sample width — lower is better. 16-bit (subslot 2) is the
// PROVEN-AUDIBLE format over the patched PIO host; native 24-bit (3) and
// 24-in-32 (4) stream but were still under investigation in the field, so we
// keep 16-bit as the default choice while accepting the wider formats when a
// DAC offers nothing else (a headset codec that only advertises 32-bit would
// otherwise read as "incompatible").
static uint8_t subslot_rank(uint8_t subslot) {
  switch (subslot) {
    case 2:  return 0;   // 16-bit: proven
    case 3:  return 1;   // 24-bit packed
    case 4:  return 2;   // 24-in-32
    default: return 0xFF;
  }
}

// The largest packet the patched PIO encoder can stage. PIO_USB_EP_SIZE is 580
// in the carried patch; a DAC alt whose wMaxPacketSize exceeds it would smash
// the encode buffer, so such an alt is rejected here rather than crashing
// core1 mid-stream. (We never fill more than one 1 ms frame anyway.)
#ifndef UAC_HOST_MAX_WIRE_PKT
#define UAC_HOST_MAX_WIRE_PKT 580u
#endif

// Accept an alt setting if it can take our stream: stereo Type-I PCM at 48 kHz
// in a 2/3/4-byte subslot. Among acceptable alts, keep the best-ranked one.
static void consider_candidate(dac_dev_t *d, const alt_cand_t *c,
                               tusb_desc_endpoint_t const *ep) {
  const uint8_t rank = subslot_rank(c->subslot);
  // Frames this alt can carry in one 1 ms packet (whole frames only).
  const uint16_t wire_bpf = (uint16_t) (DSPICO_NUM_CHANNELS * c->subslot);
  const uint16_t max_frames = wire_bpf ? (ep->wMaxPacketSize / wire_bpf) : 0;

  // Log every candidate so an "incompatible" verdict is diagnosable from the
  // UART/WebUSB log without a USB analyzer.
  dlog("DSPico host: alt itf %u.%u ch=%u subslot=%u bits=%u pcm=%d rate48=%d "
       "maxpkt=%u(%uframes) sync=%s fb=%d\n",
         c->itf, c->alt, c->channels, c->subslot, c->bits,
         (int) c->pcm, (int) c->rate_ok, ep->wMaxPacketSize, max_frames,
         ep->bmAttributes.sync == 1 ? "async" :
         ep->bmAttributes.sync == 2 ? "adaptive" :
         ep->bmAttributes.sync == 3 ? "sync" : "none",
         (int) c->have_fb);

  if (!c->pcm || !c->rate_ok) return;
  if (c->channels != DSPICO_NUM_CHANNELS) return;
  if (rank == 0xFF) return;                       // unsupported subslot width
  if (ep->wMaxPacketSize > UAC_HOST_MAX_WIRE_PKT) {
    dlog("DSPico host:   ^ rejected: maxpkt %u exceeds PIO encode limit %u\n",
           ep->wMaxPacketSize, UAC_HOST_MAX_WIRE_PKT);
    return;
  }
  // Must carry a whole 1 ms frame (48 stereo frames). A short-packet alt would
  // starve the DAC — better to keep looking (and warn if it's all we find).
  if (max_frames < DSPICO_SAMPLES_PER_MS) {
    dlog("DSPico host:   ^ note: only %u frames/pkt (< %u) — undersized\n",
           max_frames, (unsigned) DSPICO_SAMPLES_PER_MS);
    return;
  }

  if (d->have_ep && subslot_rank(d->subslot) <= rank) return;  // keep better/equal

  d->as_itf   = c->itf;
  d->as_alt   = c->alt;
  d->subslot  = c->subslot;
  d->bits     = c->bits;
  d->cand_rank = rank;
  d->ep_async = (ep->bmAttributes.sync == 1);   // 1 = asynchronous
  memcpy(&d->ep_out, ep, sizeof(*ep));
  d->have_ep = d->have_as = true;
  d->have_fb = c->have_fb;
  if (c->have_fb) memcpy(&d->ep_fb, &c->fb_ep, sizeof(d->ep_fb));
}

// UAC1 Type I format descriptor carries the supported sample rates; check that
// 48 kHz is among them (discrete list) or inside them (continuous range).
// Reads stay within the descriptor's own bLength.
static bool uac1_rate_list_has_48k(uint8_t const *p, uint8_t dlen) {
  const uint8_t n = p[7];   // bSamFreqType: 0 = continuous min..max, else count
  if (n == 0) {
    if (dlen < 14) return false;
    const uint32_t lo = (uint32_t) (p[8]  | (p[9]  << 8) | (p[10] << 16));
    const uint32_t hi = (uint32_t) (p[11] | (p[12] << 8) | (p[13] << 16));
    return lo <= DSPICO_SAMPLE_RATE_HZ && DSPICO_SAMPLE_RATE_HZ <= hi;
  }
  for (uint8_t i = 0; i < n && (uint16_t) (8 + 3 * i + 3) <= dlen; i++) {
    const uint8_t *f = p + 8 + 3 * i;
    if ((uint32_t) (f[0] | (f[1] << 8) | (f[2] << 16)) == DSPICO_SAMPLE_RATE_HZ)
      return true;
  }
  return false;
}

// Judge the current AS-alt candidate (data + optional feedback EP) and reset it
// so the next alt starts clean. Called at every AS-interface boundary and at
// the end of the block, so the feedback EP that follows the data EP is included.
static void flush_candidate(dac_dev_t *d, alt_cand_t *cand) {
  if (cand->in_as_itf && cand->have_data) {
    consider_candidate(d, cand, &cand->data_ep);
  }
  cand->in_as_itf = false;
  cand->have_data = false;
  cand->have_fb   = false;
}

// Single-pass parse of the whole audio-function descriptor block. With an IAD
// (typical UAC2) TinyUSB hands us the entire function at once; without one
// (typical UAC1) it hands us one interface at a time — walking `len` bytes and
// tracking the current interface context handles both.
//
// Records: the AC interface number, UAC version, every UAC2 clock source, a
// clock selector, the playback feature units, and — via consider_candidate() —
// the best alt setting whose Type-I PCM format we can feed (stereo 48 kHz).
static void parse_audio_function(dac_dev_t *d, uint8_t const *p, uint16_t len) {
  uint8_t const *end = p + len;
  uint8_t cur_sub = 0xFF;                 // current interface subclass
  alt_cand_t cand = { .in_as_itf = false };

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    const uint8_t dtype = p[1];
    const uint8_t dlen  = p[0];

    if (dtype == TUSB_DESC_INTERFACE) {
      flush_candidate(d, &cand);          // judge the alt we were inside
      tusb_desc_interface_t const *itf = (tusb_desc_interface_t const *) p;
      if (itf->bInterfaceClass == TUSB_CLASS_AUDIO) {
        cur_sub = itf->bInterfaceSubClass;
        if (cur_sub == AUDIO_SUBCLASS_CONTROL) {
          d->ac_itf = itf->bInterfaceNumber;
        } else if (cur_sub == AUDIO_SUBCLASS_STREAMING) {
          cand = (alt_cand_t) {
            .itf = itf->bInterfaceNumber, .alt = itf->bAlternateSetting,
            .rate_ok = true, .in_as_itf = true,
          };
        }
      } else {
        cur_sub = 0xFF;   // some non-audio interface inside the block
      }
    } else if (dtype == TUSB_DESC_CS_INTERFACE && cur_sub == AUDIO_SUBCLASS_CONTROL) {
      const uint8_t subtype = p[2];
      if (subtype == AUDIO_CS_AC_INTERFACE_HEADER) {
        d->audio_bcd = tu_unaligned_read16(p + 3);   // bcdADC
      } else if (subtype == AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE && dlen >= 4) {
        if (d->clock_count < TU_ARRAY_SIZE(d->clock_ids))
          d->clock_ids[d->clock_count++] = p[3];      // bClockID
      } else if (subtype == AUDIO_CS_AC_INTERFACE_CLOCK_SELECTOR && dlen >= 4) {
        d->clock_sel_id = p[3];                        // bClockID of the selector
      } else if (subtype == AUDIO_CS_AC_INTERFACE_FEATURE_UNIT && dlen >= 5) {
        if (d->fu_count < TU_ARRAY_SIZE(d->fu_ids)) {
          const uint8_t idx = d->fu_count++;
          d->fu_ids[idx] = p[3];   // bUnitID
          // UAC2 Feature Unit: bmaControls[ch] is 4 bytes each, starting after
          // the 5-byte header (bLength,type,subtype,bUnitID,bSourceID), one per
          // channel including master (ch0). Record the low byte per channel.
          // (UAC1 uses a bControlSize-based layout; we only build the precise
          // control map for UAC2, where the NAK-forever quirk was observed.)
          uint8_t nch = 0;
          if (d->audio_bcd >= 0x0200 && dlen >= 6) {
            nch = (uint8_t) ((dlen - 6) / 4);   // trailing byte is iFeature
            if (nch > FU_MAX_CH) nch = FU_MAX_CH;
            for (uint8_t c = 0; c < nch; c++) d->fu_ctrl[idx][c] = p[5 + c * 4];
          }
          d->fu_nch[idx] = nch;
        }
      } else if (subtype == AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL && dlen >= 8) {
        // Remember the speaker/headphone terminal's source: if that source is
        // a Feature Unit, it's the one controlling playback volume/mute.
        const uint16_t ttype = tu_unaligned_read16(p + 4);
        if ((ttype & 0xFF00) == 0x0300) d->ot_source_id = p[7];   // 0x03xx = output
      }
    } else if (dtype == TUSB_DESC_CS_INTERFACE && cand.in_as_itf) {
      const uint8_t subtype = p[2];
      const bool uac2 = d->audio_bcd >= 0x0200;
      if (subtype == AUDIO_CS_AS_INTERFACE_AS_GENERAL) {
        if (uac2 && dlen >= 16) {
          cand.pcm      = (p[6] & 0x01) != 0;   // bmFormats bit 0 = Type I PCM
          cand.channels = p[10];                // bNrChannels
        } else if (!uac2 && dlen >= 7) {
          cand.pcm = (uint16_t) (p[5] | (p[6] << 8)) == 0x0001;   // wFormatTag PCM
        }
      } else if (subtype == AUDIO_CS_AS_INTERFACE_FORMAT_TYPE && p[3] == 1 /*Type I*/) {
        if (uac2 && dlen >= 6) {
          cand.subslot = p[4];                  // bSubslotSize
          cand.bits    = p[5];                  // bBitResolution
        } else if (!uac2 && dlen >= 8) {
          cand.channels = p[4];                 // bNrChannels
          cand.subslot  = p[5];                 // bSubframeSize
          cand.bits     = p[6];                 // bBitResolution
          cand.rate_ok  = uac1_rate_list_has_48k(p, dlen);
        }
      }
    } else if (dtype == TUSB_DESC_ENDPOINT && cand.in_as_itf) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
      const bool is_iso  = (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS);
      const bool is_out  = (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_OUT);
      const bool is_data = (ep->bmAttributes.usage == 0x00);   // 0x00 = data
      const bool is_fb   = (ep->bmAttributes.usage == 0x01);   // 0x01 = feedback
      if (is_iso && is_out && is_data) {
        memcpy(&cand.data_ep, ep, sizeof(cand.data_ep));
        cand.have_data = true;
      } else if (is_iso && !is_out && is_fb) {
        memcpy(&cand.fb_ep, ep, sizeof(cand.fb_ep));
        cand.have_fb = true;
      }
    }
    p = desc_next(p);
  }
  flush_candidate(d, &cand);              // judge the final alt at block end
}

// ---------------------------------------------------------------------------
// TinyUSB host class-driver callbacks
// ---------------------------------------------------------------------------
static bool dac_init(void) {
  memset(&s_dac, 0, sizeof(s_dac));
  s_desc_len = s_desc_pos = 0;
  return true;
}

static bool dac_open(uint8_t rhport, uint8_t dev_addr,
                     tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO) return false;

  // Bind to the first audio device we see (single-DAC spike).
  if (s_dac.in_use && s_dac.dev_addr != dev_addr) return false;
  if (!s_dac.in_use) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    dlog("DSPico host: audio device attached (addr %u, VID:PID %04X:%04X)\n",
           dev_addr, vid, pid);
  }
  s_dac.in_use = true;
  s_dac.dev_addr = dev_addr;
  s_dac.rhport = rhport;

  // Capture the descriptor block for the incremental hex dump (first call only —
  // UAC1 without an IAD calls open() once per interface). Suppressed while the
  // USB device-trace is on, so the dump burst can't garble the device log.
#if !DSPICO_USB_TRACE
  if (s_desc_len == 0) {
    uint16_t n = max_len < UAC_DESC_DUMP_MAX ? max_len : UAC_DESC_DUMP_MAX;
    memcpy(s_desc_dump, itf_desc, n);
    s_desc_len = n;
    s_desc_pos = 0;
    dlog("DSPico host: dumping %u-byte audio descriptor block:\n", n);
  }
#endif

  parse_audio_function(&s_dac, (uint8_t const *) itf_desc, max_len);
  return true;   // claim the audio interface(s) in this block
}

// --- Setup state machine (runs after enumeration) ---------------------------
// Step order matters and differs by UAC version (matching what Linux/Windows
// do, which is what DACs are tested against):
//   UAC2: SET_CUR(clock = 48 kHz) FIRST (alt still 0), then SET_INTERFACE(alt)
//         — many DACs reject or ignore a rate change on an already-active alt.
//   UAC1: SET_INTERFACE(alt) first, then the endpoint SAMPLING_FREQ control
//         — the endpoint only exists once the alt is active.
static void submit_first_packet(void);
static void on_set_alt_complete(tuh_xfer_t *xfer);
static void on_set_freq_complete(tuh_xfer_t *xfer);

// --- Setup-chain watchdog ----------------------------------------------------
// Some DACs NAK forever instead of STALLing a request they dislike; pio_usb
// retries NAKs indefinitely and TinyUSB has no control-transfer timeout, so a
// single such request used to wedge the whole chain (seen in the field: a
// UAC1 sample-rate SET_CUR and a post-STALL Feature-Unit write that never
// completed). Every chain step arms a deadline; uac_host_task() — called from
// the core1 loop — aborts the stuck transfer and runs the step's timeout
// continuation, so setup always terminates.
#define SETUP_STEP_TIMEOUT_MS 600u

static void (*s_timeout_next)(void);
static uint32_t s_step_deadline_ms;        // 0 = no step pending

// Generation token: every chain submit carries it in user_data. An aborted
// transfer's completion event can still arrive later (abort vs. completion
// race in the frame IRQ); callbacks ignore anything from an old generation
// so the chain can't be double-driven.
static uint32_t s_chain_gen;

// After an abort, let the bus/stack settle a couple of frames before the next
// step instead of submitting from inside the same task pass. The same
// mechanism paces EVERY chain step by UAC_HOST_CTL_PACE_MS (the Linux
// CTL_MSG_DELAY quirk that quirky USB-C dongle DACs need — see the top of the
// file): completion handlers hand the next step to schedule_step() instead of
// calling it inline.
static void (*s_deferred_next)(void);
static uint32_t s_deferred_at_ms;

static void schedule_step(void (*fn)(void)) {
  s_deferred_next = fn;
  s_deferred_at_ms = to_ms_since_boot(get_absolute_time()) + UAC_HOST_CTL_PACE_MS;
}

static void arm_step_watchdog(void (*on_timeout)(void)) {
  s_timeout_next = on_timeout;
  uint32_t dl = to_ms_since_boot(get_absolute_time()) + SETUP_STEP_TIMEOUT_MS;
  s_step_deadline_ms = dl ? dl : 1;
}
static void disarm_step_watchdog(void) { s_step_deadline_ms = 0; }

// True (and logs) when a completion belongs to an aborted/older submit.
static bool stale_completion(tuh_xfer_t *xfer) {
  if ((uint32_t) xfer->user_data == s_chain_gen) return false;
  dlog("DSPico host: stale completion ignored\n");
  return true;
}

void uac_host_task(void) {
  const uint32_t now = to_ms_since_boot(get_absolute_time());

  // Drip the captured descriptor block into the diagnostic log, one 16-byte
  // line per pass so the log ring never overflows.
  if (s_desc_pos < s_desc_len) {
    char line[64];
    int  n = 0;
    n += snprintf(line + n, sizeof line - n, "  %03u:", (unsigned) s_desc_pos);
    for (uint16_t i = 0; i < 16 && s_desc_pos + i < s_desc_len; i++) {
      n += snprintf(line + n, sizeof line - n, " %02X", s_desc_dump[s_desc_pos + i]);
    }
    dlog("%s\n", line);
    s_desc_pos += 16;
    return;   // one line per pass
  }

  if (s_deferred_next && (int32_t) (now - s_deferred_at_ms) >= 0) {
    void (*fn)(void) = s_deferred_next;
    s_deferred_next = NULL;
    fn();
  }

  if (s_step_deadline_ms == 0 || !s_dac.in_use) return;
  if ((int32_t) (now - s_step_deadline_ms) < 0) return;
  s_step_deadline_ms = 0;
  dlog("DSPico host: control step TIMED OUT (device NAKing?) — aborting + continuing\n");
  s_chain_gen++;                              // invalidate any late completion
  tuh_edpt_abort_xfer(s_dac.dev_addr, 0);
  s_deferred_next = s_timeout_next;           // continue after ~2 frames
  s_timeout_next = NULL;
  s_deferred_at_ms = now + 20;
}

// Timeout / submit-failure continuations — same forward path the completion
// handlers take on failure, so a dead step can never strand the chain.
static bool setup_is_uac2(void);
static void send_set_interface(void);
static void start_fu_init(void);
static void fu_send_next(void);
static void finish_setup(void);

static void timeout_alt(void) {
  // Without the operational alt there is nothing to stream to.
  s_dac.incompatible = true;
  usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
}
static void send_set_sample_rate_uac1(void);
static void send_set_rate_current_clock(void);   // UAC2: one clock source
static void send_get_rate_readback(void);         // UAC2: verify + log
static uint8_t s_rate_tries;
static uint8_t s_clock_idx;   // which UAC2 clock source we're programming

static void timeout_rate(void) {
  // A NAK-forever rate request may still succeed on a retry once the DAC has
  // settled (the audible symptom of an unset rate is wrong-pitch playback) —
  // try a couple more times before moving on to the next clock / step.
  if (++s_rate_tries < 3) {
    dlog("DSPico host: retrying sample-rate set (%u/3)\n", (unsigned) (s_rate_tries + 1));
    if (setup_is_uac2()) send_set_rate_current_clock();
    else                 send_set_sample_rate_uac1();
    return;
  }
  if (setup_is_uac2()) {
    // Give up on this clock source; try the next, else read back + continue.
    s_rate_tries = 0;
    if (++s_clock_idx < s_dac.clock_count) schedule_step(send_set_rate_current_clock);
    else                                   schedule_step(send_get_rate_readback);
  } else {
    start_fu_init();
  }
}
static void timeout_fu(void) {
  fu_send_next();   // s_fu_step already advanced — moves to the next control
}

static bool setup_is_uac2(void) {
  return s_dac.audio_bcd >= 0x0200 && s_dac.clock_count > 0;
}

// --- DAC Feature-Unit init (unmute + 0 dB volume) ----------------------------
// Real hosts set the playback Feature Unit's mute/volume right after
// enumeration, and DACs are tested against that — some power up muted or at
// minimum volume and stay silent forever if nobody does it. The wire format is
// the same for UAC1 (SET_CUR) and UAC2 (CUR): bRequest 0x01, wValue =
// (selector << 8) | channel, wIndex = (fu_id << 8) | ac_itf.
//
// CRUCIAL (CX31988): send a control ONLY where the FU's bmaControls says it
// exists. That codec carries Mute on the master and Volume on L/R only, and
// NAKs FOREVER (never STALLs) on a write to a control it lacks — the NAK storm
// wedged the very L/R volume writes that raise it off its minimum, so it
// streamed silently. The step list is now built from the parsed control map.
static void finish_setup(void);

// Volume target: a modest -12 dB (1/256 dB units) rather than 0 dB — audible
// on every DAC, but doesn't slam a bus-powered amp to full output the moment
// it unmutes (marginal VBUS + full amp draw can brown the dongle out into an
// attach/detach loop). The OS volume on the PC side still scales the stream.
#define FU_VOLUME_TARGET ((uint16_t)(int16_t)(-12 * 256))

// The step list, built by build_fu_steps() from the resolved FU's control map.
// Room for master + stereo, each with a mute and a volume control.
typedef struct { uint8_t sel, ch, len; uint16_t val; } fu_step_t;
static fu_step_t s_fu_steps[FU_MAX_CH * 2];
static uint8_t   s_fu_nsteps;
static uint8_t   s_fu_step;
static uint8_t   s_fu_buf[2];

static void fu_send_next(void);

// Turn the resolved FU's control map into the concrete SET_CUR steps to send.
// For UAC1 (no map parsed) fall back to trying mute+volume on master + stereo,
// which is what real UAC1 hosts do and what earlier field DACs tolerated.
static void build_fu_steps(void) {
  s_fu_nsteps = 0;
  s_fu_step   = 0;
  if (s_dac.fu_ctrl_nch == 0) {           // UAC1 / no control map — try the lot
    for (uint8_t ch = 0; ch < FU_MAX_CH; ch++)
      s_fu_steps[s_fu_nsteps++] = (fu_step_t){ AUDIO_FU_CTRL_MUTE, ch, 1, 0 };
    for (uint8_t ch = 0; ch < FU_MAX_CH; ch++)
      s_fu_steps[s_fu_nsteps++] = (fu_step_t){ AUDIO_FU_CTRL_VOLUME, ch, 2, FU_VOLUME_TARGET };
    return;
  }
  // UAC2: emit only controls the FU actually implements (bits 0-1 Mute,
  // bits 2-3 Volume; non-zero = present).
  for (uint8_t ch = 0; ch < s_dac.fu_ctrl_nch; ch++)
    if (s_dac.fu_ctrl_sel[ch] & 0x03)
      s_fu_steps[s_fu_nsteps++] = (fu_step_t){ AUDIO_FU_CTRL_MUTE, ch, 1, 0 };
  for (uint8_t ch = 0; ch < s_dac.fu_ctrl_nch; ch++)
    if (s_dac.fu_ctrl_sel[ch] & 0x0C)
      s_fu_steps[s_fu_nsteps++] = (fu_step_t){ AUDIO_FU_CTRL_VOLUME, ch, 2, FU_VOLUME_TARGET };
}

static void on_fu_step_complete(tuh_xfer_t *xfer) {
  if (stale_completion(xfer)) return;
  disarm_step_watchdog();
  const uint8_t i = s_fu_step - 1;
  dlog("DSPico host: FU %u %s ch%u -> %s\n", s_dac.fu_id,
         s_fu_steps[i].sel == AUDIO_FU_CTRL_MUTE ? "unmute" : "vol -12dB",
         s_fu_steps[i].ch,
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "stalled (not implemented)");
  schedule_step(fu_send_next);   // pace control requests (dongle quirk)
}

static void fu_send_next(void) {
  if (s_dac.fu_id == 0 || s_fu_step >= s_fu_nsteps) {
    finish_setup();
    return;
  }
  const uint8_t i = s_fu_step++;
  s_fu_buf[0] = (uint8_t) (s_fu_steps[i].val & 0xFF);
  s_fu_buf[1] = (uint8_t) (s_fu_steps[i].val >> 8);

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,   // == UAC1 SET_CUR (0x01)
      .wValue = tu_htole16((uint16_t) ((s_fu_steps[i].sel << 8) | s_fu_steps[i].ch)),
      .wIndex = tu_htole16((uint16_t) ((s_dac.fu_id << 8) | s_dac.ac_itf)),
      .wLength = tu_htole16(s_fu_steps[i].len),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_fu_buf,
      .complete_cb = on_fu_step_complete,
      .user_data = ++s_chain_gen,
  };
  if (tuh_control_xfer(&xfer)) {
    arm_step_watchdog(timeout_fu);
  } else {
    // Submit refused (control pipe busy/gone): don't strand the chain — the
    // stream matters more than the volume preset.
    dlog("DSPico host: FU step submit failed — skipping volume init\n");
    finish_setup();
  }
}

static void start_fu_init(void) {
  build_fu_steps();
  if (s_dac.fu_id) {
    dlog("DSPico host: initialising DAC Feature Unit %u — %u controls to set "
         "(map: m%c/v%c m%c/v%c m%c/v%c)\n",
           s_dac.fu_id, s_fu_nsteps,
           (s_dac.fu_ctrl_nch > 0 && (s_dac.fu_ctrl_sel[0] & 0x03)) ? 'Y' : '-',
           (s_dac.fu_ctrl_nch > 0 && (s_dac.fu_ctrl_sel[0] & 0x0C)) ? 'Y' : '-',
           (s_dac.fu_ctrl_nch > 1 && (s_dac.fu_ctrl_sel[1] & 0x03)) ? 'Y' : '-',
           (s_dac.fu_ctrl_nch > 1 && (s_dac.fu_ctrl_sel[1] & 0x0C)) ? 'Y' : '-',
           (s_dac.fu_ctrl_nch > 2 && (s_dac.fu_ctrl_sel[2] & 0x03)) ? 'Y' : '-',
           (s_dac.fu_ctrl_nch > 2 && (s_dac.fu_ctrl_sel[2] & 0x0C)) ? 'Y' : '-');
  } else {
    dlog("DSPico host: no Feature Unit found — skipping volume init\n");
  }
  fu_send_next();
}

// Per-stream packet stats for the heartbeat (reset at every stream start).
static uint32_t s_iso_pkts;
static uint32_t s_stream_t0_ms;

// Final step for both protocols: open the iso OUT endpoint and start pumping.
static void finish_setup(void) {
  disarm_step_watchdog();
  if (tuh_edpt_open(s_dac.dev_addr, &s_dac.ep_out)) {
    s_iso_pkts = 0;
    s_stream_t0_ms = to_ms_since_boot(get_absolute_time());
    s_dac.streaming = true;
    dlog("DSPico host: streaming started (EP 0x%02X, %u-byte subslot)\n",
           s_dac.ep_out.bEndpointAddress, s_dac.subslot);
    submit_first_packet();
  } else {
    s_dac.incompatible = true;
    dlog("DSPico host: ERROR — tuh_edpt_open(0x%02X) failed\n",
           s_dac.ep_out.bEndpointAddress);
  }
  usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
}

// SET_INTERFACE(as_itf, as_alt) — activates the iso endpoint on the DAC.
static void send_set_interface(void) {
  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_STANDARD,
                             .direction = TUSB_DIR_OUT },
      .bRequest = TUSB_REQ_SET_INTERFACE,
      .wValue = tu_htole16(s_dac.as_alt),
      .wIndex = tu_htole16(s_dac.as_itf),
      .wLength = 0,
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = NULL,
      .complete_cb = on_set_alt_complete,
      .user_data = ++s_chain_gen,
  };
  if (tuh_control_xfer(&xfer)) {
    arm_step_watchdog(timeout_alt);
  } else {
    dlog("DSPico host: SET_INTERFACE submit failed\n");
    timeout_alt();
  }
}

// UAC2: SET_CUR sample frequency (SAM_FREQ) on ONE Clock Source entity. Headset
// codecs expose several clock sources (playback + capture paths) and sometimes a
// Clock Selector; we program every source in turn (send_set_rate_current_clock
// walks s_clock_idx) rather than betting on one. A source that rejects the rate
// STALLs — fine, we move to the next.
static uint8_t s_freq_buf[4];
static void send_set_rate_current_clock(void) {
  s_freq_buf[0] = (uint8_t)(DSPICO_SAMPLE_RATE_HZ & 0xFF);
  s_freq_buf[1] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 8) & 0xFF);
  s_freq_buf[2] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 16) & 0xFF);
  s_freq_buf[3] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 24) & 0xFF);

  const uint8_t clk = s_dac.clock_ids[s_clock_idx];
  dlog("DSPico host: SET rate 48000 on clock source %u (%u/%u)\n",
         clk, (unsigned) (s_clock_idx + 1), (unsigned) s_dac.clock_count);

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,
      .wValue = tu_htole16((uint16_t)(AUDIO_CS_CTRL_SAM_FREQ << 8)),
      .wIndex = tu_htole16((uint16_t)((clk << 8) | s_dac.ac_itf)),
      .wLength = tu_htole16(4),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_freq_buf,
      .complete_cb = on_set_freq_complete,
      .user_data = ++s_chain_gen,
  };
  if (tuh_control_xfer(&xfer)) {
    arm_step_watchdog(timeout_rate);
  } else {
    dlog("DSPico host: set-rate submit failed\n");
    timeout_rate();
  }
}

// UAC2: GET_CUR the sample frequency back from the first clock source so the log
// shows the rate the DAC actually settled on — the decisive datapoint for the
// "plays at the wrong pitch" symptom (a NAK-forever SET leaves the DAC at its
// power-on default, which the readback exposes without a USB analyzer).
static void on_get_rate_complete(tuh_xfer_t *xfer);
static void send_get_rate_readback(void) {
  const uint8_t clk = s_dac.clock_ids[0];
  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_INTERFACE,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_IN },
      .bRequest = AUDIO_CS_REQ_CUR,
      .wValue = tu_htole16((uint16_t)(AUDIO_CS_CTRL_SAM_FREQ << 8)),
      .wIndex = tu_htole16((uint16_t)((clk << 8) | s_dac.ac_itf)),
      .wLength = tu_htole16(4),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_freq_buf,
      .complete_cb = on_get_rate_complete,
      .user_data = ++s_chain_gen,
  };
  if (tuh_control_xfer(&xfer)) {
    arm_step_watchdog(send_set_interface);   // on timeout, just proceed to alt
  } else {
    send_set_interface();
  }
}

static void on_get_rate_complete(tuh_xfer_t *xfer) {
  if (stale_completion(xfer)) return;
  disarm_step_watchdog();
  if (xfer->result == XFER_RESULT_SUCCESS) {
    const uint32_t rate = (uint32_t) (s_freq_buf[0] | (s_freq_buf[1] << 8) |
                                      (s_freq_buf[2] << 16) | ((uint32_t) s_freq_buf[3] << 24));
    dlog("DSPico host: clock reads back %lu Hz (want %lu)%s\n",
           (unsigned long) rate, (unsigned long) DSPICO_SAMPLE_RATE_HZ,
           rate == DSPICO_SAMPLE_RATE_HZ ? "" : " — MISMATCH (wrong pitch expected)");
  } else {
    dlog("DSPico host: clock rate readback stalled (continuing)\n");
  }
  schedule_step(send_set_interface);
}

// UAC1: SET_CUR sampling frequency (SAMPLING_FREQ_CONTROL) on the iso endpoint.
static void send_set_sample_rate_uac1(void) {
  s_freq_buf[0] = (uint8_t)(DSPICO_SAMPLE_RATE_HZ & 0xFF);
  s_freq_buf[1] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 8) & 0xFF);
  s_freq_buf[2] = (uint8_t)((DSPICO_SAMPLE_RATE_HZ >> 16) & 0xFF);

  tusb_control_request_t const req = {
      .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_ENDPOINT,
                             .type = TUSB_REQ_TYPE_CLASS,
                             .direction = TUSB_DIR_OUT },
      .bRequest = AUDIO_CS_REQ_CUR,
      // UAC1 endpoint control selector: SAMPLING_FREQ_CONTROL = 0x01.
      .wValue = tu_htole16((uint16_t)(0x01 << 8)),
      .wIndex = tu_htole16(s_dac.ep_out.bEndpointAddress),
      .wLength = tu_htole16(3),
  };
  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = 0,
      .setup = &req,
      .buffer = s_freq_buf,
      .complete_cb = on_set_freq_complete,
      .user_data = ++s_chain_gen,
  };
  if (tuh_control_xfer(&xfer)) {
    arm_step_watchdog(timeout_rate);
  } else {
    dlog("DSPico host: set-rate submit failed\n");
    timeout_rate();
  }
}

static bool dac_set_config(uint8_t dev_addr, uint8_t itf_num) {
  // We only drive the streaming interface; report the control interface as
  // immediately configured.
  if (!s_dac.have_as || itf_num != s_dac.as_itf) {
    if (!s_dac.have_as && itf_num == s_dac.ac_itf) {
      s_dac.incompatible = true;
      dlog("DSPico host: DAC has no stereo 48 kHz Type-I PCM alt (16/24-bit) — not streaming\n");
    }
    usbh_driver_set_config_complete(dev_addr, itf_num);
    return true;
  }

  dlog("DSPico host: DAC (UAC%c) itf %u alt %u — %u-bit in %u-byte subslot, EP 0x%02X "
       "maxpkt %u sync=%s%s\n",
         setup_is_uac2() ? '2' : '1',
         s_dac.as_itf, s_dac.as_alt, s_dac.bits, s_dac.subslot,
         s_dac.ep_out.bEndpointAddress, s_dac.ep_out.wMaxPacketSize,
         s_dac.ep_async ? "async" : "adaptive/sync",
         s_dac.subslot == 2 ? " (16-bit, truncating)" :
         s_dac.subslot == 4 ? " (24-in-32, left-justified)" : "");
  if (s_dac.clock_count > 1 || s_dac.clock_sel_id)
    dlog("DSPico host: clock topology — %u sources, selector id %u\n",
           s_dac.clock_count, s_dac.clock_sel_id);
  if (s_dac.ep_async && s_dac.have_fb)
    dlog("DSPico host: DAC async feedback EP 0x%02X present (host sends fixed 48/frame; "
         "drift tolerated by DAC buffering)\n", s_dac.ep_fb.bEndpointAddress);

  // Resolve the playback Feature Unit for the volume-init step: prefer the
  // unit that feeds the speaker/headphone Output Terminal, else the first.
  s_dac.fu_id = 0;
  uint8_t fu_idx = 0xFF;
  for (uint8_t i = 0; i < s_dac.fu_count; i++) {
    if (s_dac.fu_ids[i] == s_dac.ot_source_id) { s_dac.fu_id = s_dac.ot_source_id; fu_idx = i; break; }
  }
  if (s_dac.fu_id == 0 && s_dac.fu_count > 0) { s_dac.fu_id = s_dac.fu_ids[0]; fu_idx = 0; }

  // Copy the chosen FU's control map so the step builder sends only controls
  // that exist (see build_fu_steps / the CX31988 note).
  s_dac.fu_ctrl_nch = 0;
  if (fu_idx != 0xFF) {
    s_dac.fu_ctrl_nch = s_dac.fu_nch[fu_idx];
    memcpy(s_dac.fu_ctrl_sel, s_dac.fu_ctrl[fu_idx], FU_MAX_CH);
  }

  // Kick the setup chain (see the ordering note above).
  s_rate_tries = 0;
  s_clock_idx  = 0;
  if (setup_is_uac2()) {
    send_set_rate_current_clock(); // rate on every clock source, then alt
  } else {
    send_set_interface();          // alt first, then endpoint rate
  }
  return true;
}

static void on_set_alt_complete(tuh_xfer_t *xfer) {
  if (stale_completion(xfer)) return;
  disarm_step_watchdog();
  dlog("DSPico host: SET_INTERFACE(itf %u, alt %u) -> %s\n",
         s_dac.as_itf, s_dac.as_alt,
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "FAILED");
  if (xfer->result != XFER_RESULT_SUCCESS) {
    s_dac.incompatible = true;
    usbh_driver_set_config_complete(s_dac.dev_addr, s_dac.as_itf);
    return;
  }
  if (setup_is_uac2()) {
    schedule_step(start_fu_init);            // UAC2: rate set — unmute, then stream
  } else {
    schedule_step(send_set_sample_rate_uac1);// UAC1: rate lives on the (now active) EP
  }
}

static void on_set_freq_complete(tuh_xfer_t *xfer) {
  if (stale_completion(xfer)) return;
  disarm_step_watchdog();
  // A DAC that only supports 48 kHz may STALL the set-rate request; that's
  // fine — its fixed rate is the one we want anyway. Log it either way.
  dlog("DSPico host: SET sample rate 48000 -> %s\n",
         xfer->result == XFER_RESULT_SUCCESS ? "OK" : "stalled/failed (continuing)");
  if (setup_is_uac2()) {
    s_rate_tries = 0;
    if (++s_clock_idx < s_dac.clock_count) {
      schedule_step(send_set_rate_current_clock);   // program the next clock source
    } else {
      schedule_step(send_get_rate_readback);        // all sources done — verify + log
    }
  } else {
    schedule_step(start_fu_init);  // UAC1: alt+rate done — unmute, then stream
  }
}

// --- Isochronous streaming loop --------------------------------------------
static uint16_t build_packet(void) {
  const uint8_t  wire_bps = s_dac.subslot;                       // 2, 3 or 4
  const uint16_t wire_bpf = DSPICO_NUM_CHANNELS * wire_bps;      // bytes/frame on the wire

  size_t frames = DSPICO_SAMPLES_PER_MS;
  const size_t max_frames = s_dac.ep_out.wMaxPacketSize / wire_bpf;
  if (frames > max_frames) frames = max_frames;

  // The source always produces the native 24-bit/3-byte format (6 B/frame).
  size_t got = 0;
  if (s_fill_cb) got = s_fill_cb(s_pkt, frames);
  if (got == 0) {
    // No source / underrun: send a silent packet to keep the stream alive.
    memset(s_pkt, 0, frames * DSPICO_NUM_CHANNELS * DSPICO_BYTES_PER_SAMPLE);
    got = frames;
  }

  const size_t samples = got * DSPICO_NUM_CHANNELS;
  if (wire_bps == 2) {
    // 16-bit alt: truncate each 24-bit LE sample to its top 16 bits. Forward
    // in place is safe — the 2-byte destination always trails the 3-byte source.
    for (size_t s = 0; s < samples; s++) {
      s_pkt[s * 2 + 0] = s_pkt[s * 3 + 1];
      s_pkt[s * 2 + 1] = s_pkt[s * 3 + 2];
    }
  } else if (wire_bps == 4) {
    // 24-in-32 alt: place the 24-bit sample left-justified (MSB-aligned) in a
    // 4-byte LE slot with the unused low byte zero, per USB Frmts20 §2.3.1.7.1.
    // Expanding grows the buffer, so walk it BACKWARDS (the 4-byte destination
    // leads the 3-byte source) to avoid clobbering samples not yet copied.
    for (size_t s = samples; s-- > 0; ) {
      const uint8_t lsb = s_pkt[s * 3 + 0];
      const uint8_t mid = s_pkt[s * 3 + 1];
      const uint8_t msb = s_pkt[s * 3 + 2];
      s_pkt[s * 4 + 0] = 0;
      s_pkt[s * 4 + 1] = lsb;
      s_pkt[s * 4 + 2] = mid;
      s_pkt[s * 4 + 3] = msb;
    }
  }
  return (uint16_t)(got * wire_bpf);
}

static void on_iso_complete(tuh_xfer_t *xfer);

static void submit_packet(void) {
  if (!s_dac.streaming) return;
  const uint16_t len = build_packet();

  tuh_xfer_t xfer = {
      .daddr = s_dac.dev_addr,
      .ep_addr = s_dac.ep_out.bEndpointAddress,
      .buflen = len,
      .buffer = s_pkt,
      .complete_cb = on_iso_complete,
      .user_data = 0,
  };
  tuh_edpt_xfer(&xfer);
}

static void submit_first_packet(void) { submit_packet(); }

// Iso has no retransmit; even on a (rare) error we keep the cadence going by
// immediately queuing the next frame. A periodic heartbeat on the UART proves
// packets are actually flowing (~1000/s expected at 48 kHz).
static void on_iso_complete(tuh_xfer_t *xfer) {
  if (xfer->daddr != s_dac.dev_addr) return;
#if !DSPICO_USB_TRACE
  if ((++s_iso_pkts & 0x1FFF) == 0) {   // every 8192 packets ≈ 8 s
    // Rate must sit at ~1000 pkts/s (one per USB frame). Ring fill says where
    // a silence problem lives: ~0 with the PC playing means PC audio isn't
    // arriving (device side); large+stable means the path is healthy and any
    // silence is downstream of us.
    const uint32_t ms = to_ms_since_boot(get_absolute_time()) - s_stream_t0_ms;
    dlog("DSPico host: heartbeat — %lu pkts this stream (%lu pkts/s), play ring %lu B\n",
           (unsigned long) s_iso_pkts,
           (unsigned long) (ms ? (uint64_t) s_iso_pkts * 1000u / ms : 0),
           (unsigned long) signal_path_play_fill());
  }
#endif
  submit_packet();
}

// Endpoints opened via tuh_edpt_open() report completion through the per-xfer
// complete_cb above, so the class-driver xfer_cb has nothing to do here.
static bool dac_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result,
                        uint32_t xferred_bytes) {
  (void) dev_addr; (void) ep_addr; (void) result; (void) xferred_bytes;
  return true;
}

static void dac_close(uint8_t dev_addr) {
  if (dev_addr != s_dac.dev_addr) return;
  disarm_step_watchdog();
  s_deferred_next = NULL;
  dlog("DSPico host: DAC disconnected\n");
  memset(&s_dac, 0, sizeof(s_dac));
  s_desc_len = s_desc_pos = 0;   // re-dump the descriptor on the next attach
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------
static const usbh_class_driver_t s_dac_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name       = "UAC_OUT",   // this field only exists in debug builds
#endif
    .init       = dac_init,
    .open       = dac_open,
    .set_config = dac_set_config,
    .xfer_cb    = dac_xfer_cb,
    .close      = dac_close,
};

usbh_class_driver_t const *usbh_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &s_dac_driver;
}
