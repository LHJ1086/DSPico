// ---------------------------------------------------------------------------
// DSPico — WebUSB configuration channel (device side, brief §6b).
//
// Answers the vendor control protocol from the browser app (web/dspico.js):
// device info, get/set pre-gain and bands, commit-to-flash, reset. Also serves
// the WebUSB landing-page URL and the MS OS 2.0 descriptor set (for driverless
// WinUSB on Windows). Everything runs on core0, next to the DSP it configures.
// ---------------------------------------------------------------------------
#ifndef DSPICO_CONFIG_USB_H
#define DSPICO_CONFIG_USB_H

// Load any flash-persisted EQ into the signal path. Call once at boot, after
// signal_path_init().
void config_usb_init(void);

// Run deferred config work (currently: the flash commit requested by
// REQ_COMMIT). Call from the core0 main loop — the sector erase + program is
// far too slow to run inside the USB control callback.
void config_usb_task(void);

#endif // DSPICO_CONFIG_USB_H
