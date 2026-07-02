# RP2350-USB-C — Hardware Fix Sheet

Physical board modifications for the USB-to-USB PEQ bridge, for the
**Waveshare RP2350-USB-C**. This is the bench companion to the build brief —
it covers only what you might solder/verify on the board itself.

> **Golden rule:** don't modify anything until the software tells you to.
> Flash Phase 0 first. Only do the mod below **if the PIO/host port fails to
> enumerate the DAC.** If it works as-shipped, leave the board alone.

---

## The one likely fix — remove the D+ pull-up (R13)

**Why.** The PIO USB port (Type_C2) ships wired "device-leaning": it has a
1.5 kΩ pull-up on **D+ → 3V3**. A D+ pull-up is how a *device* announces itself.
Your bridge uses this port as a **host** (it drives the DAC), and that pull-up
fights the DAC's own pull-up and corrupts the idle line state, so host
enumeration is flaky or never happens.

**What to remove on the RP2350-USB-C:** **R13** (the populated 1.5 kΩ, D+ → 3V3).

| Board variant        | Populated 1.5 kΩ D+ pull-up | Empty footprint |
|----------------------|-----------------------------|-----------------|
| **RP2350-USB-C** (this one) | **R13** — remove this | R10 (NC)        |
| RP2350-USB-CM        | R10                         | R13 (NC)        |
| RP2350-USB-A (qsantos)| R13                        | —               |

> Waveshare swaps these designators between variants, so trust the *net*, not
> the label: it is always the **~1.5 kΩ resistor between D+ and 3V3**.

**Optional, if still flaky after removing R13:** add **~15 kΩ pull-downs** from
**D+ → GND** and **D− → GND**. This gives the host the defined low idle state it
expects and fixes missed hot-plug (connect/disconnect) events.

---

## Verify with a meter BEFORE desoldering

1. Power off / unplug the board.
2. Confirm **R13 reads ~1.5 kΩ** between **GPIO13 (D+)** and the **3V3** rail.
3. Confirm **R10's pads are bare** (no part fitted).
4. If your board revision doesn't match (designators shuffle between revs):
   probe for whichever small resistor sits between the Type_C2 D+ line and 3V3
   and reads ~1.5 kΩ — **that** is the one to remove, regardless of its number.

Then desolder R13 (fine-tip iron or hot tweezers; it's a small SMD part).

---

## Do NOT touch these — they're already correct

- **R11 / R12 = 27 Ω** — series termination on D+/D−. Leave in place.
- **CC pull-downs R18 / R19 = 5.1 kΩ** (R20 = NC). PIO-USB bit-bangs the data
  lines and does its own role logic, so CC negotiation isn't used for the data
  path. Only relevant if a **Type-C** DAC won't detect the port as a source —
  see Power note below; a captive/Type-A-cabled DAC avoids it.
- **Native port (Type_C1) CC = R5 / R6 = 5.1 kΩ** — correct sink presentation to
  the PC. Leave alone.

---

## Firmware pin config (not a solder fix, but must match the board)

- **D+ = GPIO13, D− = GPIO12** on the PIO port.
- D+ is the **higher-numbered** pin — the *reverse* of Pico-PIO-USB's default
  (dp, dp+1) order. Set the library's pin / pinout-swap option so polarity is
  correct, or it will not enumerate. This is a config setting, not a board mod.
  DSPico sets this via `DSPICO_PIO_USB_PINOUT_DPDM_SWAP` in `src/board_config.h`.

---

## Power — pick the DAC to avoid a hardware problem

- Both VBUS lines share the board's **VSYS → RT9013 LDO → 3V3** rail. The PC on
  Type_C1 powers the board, and ~5 V back-feeds Type_C2's VBUS.
- There is **no VBUS switch, current limit, or protection** on the PIO port.
- **Use a self-powered DAC** (its own supply). A bus-powered DAC draws through
  the board from the PC's budget — risky and current-limited. Choosing a
  self-powered DAC removes the whole concern with zero soldering.

---

## Decision flow

```
Flash Phase 0 (host+device baseline)
        │
        ▼
Does the PIO/host port enumerate a plugged-in device?
        │
   ┌────┴────┐
  YES        NO
   │          │
 leave     remove R13 (verify with meter first)
 board      │
 as-is      ▼
        still not detected / misses hot-plug?
                 │
                 ▼
        add ~15 kΩ pull-downs on D+ and D−
                 │
                 ▼
        still failing? → suspect Type-C CC / cable, or
        fall back to ESP32-P4 / Pi per brief §10
```

---

## Bench kit for this step
- Multimeter (continuity + resistance).
- Fine-tip soldering iron or hot-air/tweezers; 15 kΩ 0402/0603 resistors if adding pull-downs.
- One known-good, **self-powered**, class-compliant USB DAC for the enumeration test.
- (Later, for audio verification) logic analyzer / USB protocol analyzer.
