// ---------------------------------------------------------------------------
// DSPico — USB descriptor layout (device side, toward the PC)
//
// Phase 1a composite (brief §8): a UAC2 speaker (OUT only) with a Feature Unit
// exposing Volume + Mute. No capture interface. The WebUSB vendor interface is
// added in Phase 3.
//
// Topology inside the Audio Control interface:
//
//   USB OUT stream ->[Input Term 0x01]->[Feature Unit 0x02]->[Output Term 0x03]
//                          ^ clocked by [Clock Source 0x04] (fixed 48 kHz)
// ---------------------------------------------------------------------------
#ifndef DSPICO_USB_DESCRIPTORS_H
#define DSPICO_USB_DESCRIPTORS_H

// --- Interface numbers -----------------------------------------------------
enum {
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_AUDIO_STREAMING,
  ITF_NUM_TOTAL
};

// --- Audio entity IDs (unique within the AC interface) ----------------------
#define UAC2_ENTITY_CLOCK        0x04
#define UAC2_ENTITY_INPUT_TERM   0x01
#define UAC2_ENTITY_FEATURE_UNIT 0x02
#define UAC2_ENTITY_OUTPUT_TERM  0x03

// --- Endpoint addresses ----------------------------------------------------
#define EPNUM_AUDIO_OUT  0x01   // isochronous data OUT (PC -> bridge)
#define EPNUM_AUDIO_FB   0x81   // isochronous feedback IN (bridge -> PC)

// --- Total length of the audio function descriptor block --------------------
// This MUST equal the sum of the per-descriptor TUD_AUDIO_DESC_*_LEN macros
// used in usb_descriptors.c. It is needed here (not there) because
// tusb_config.h consumes it for CFG_TUD_AUDIO_FUNC_1_DESC_LEN *before* the
// TinyUSB LEN macros are visible. usb_descriptors.c contains a TU_VERIFY_STATIC
// that fails the build if this literal ever drifts from the real block.
//
//   IAD 8 + STD_AC 9 + CS_AC 9 + CLK 8 + IN_TERM 17 + FEATURE(2ch) 18
//   + OUT_TERM 12 + STD_AS(alt0) 9 + STD_AS(alt1) 9 + CS_AS 16
//   + TYPE_I 6 + STD_ISO_EP 7 + CS_ISO_EP 8 + STD_ISO_FB_EP 7 = 143
#define DSPICO_UAC2_DESC_TOTAL_LEN  143

#endif // DSPICO_USB_DESCRIPTORS_H
