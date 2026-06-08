// SuperOS-606 — front-panel decode layer
//
// Confirmed by hardware. Polarity is
// uniform: a closed/active/pressed cell reads as 1 (inputs are plain INPUT, no
// pull-ups — keep it that way to preserve this).
//
// Matrix layout (drive one PH select LOW, read PB for steps / PA for rotaries+buttons;
// read PA with ALL selects HIGH for the "status-high" group):
//
//   steps 1-16      : PH[(n)/4] x PB[(n)%4]            (n = 0..15)
//   step LEDs 1-16  : PH[(n)/4] x PG[(n)%4]            (same grid, PG side)
//   MODE            : PH0 x PA0 = WRITE flag, PH0 x PA1 = TRACK flag
//   SCALE (1-4)     : PH1 x {PA0,PA1}   -> 2-bit value (scale number - 1)
//   INSTRUMENT      : PH2 x {PA0,PA1,PA2} -> 3-bit value (0=accent .. 7=closed hat)
//   FUNCTION        : PH3 x PA1
//   PATTERN CLEAR   : PH3 x PA0
//   PATTERN GROUP   : PH3 x PA2
//   WRITE/NEXT/TAP  : status-high PA1   (also the bootloader entry button)
//   RUN/STOP        : hardware only (not in the matrix)
#pragma once
#include "pins.h"
#include "hw.h"

enum Mode : uint8_t {
  PATTERN_PLAY  = 0,   // write=0 track=0
  PATTERN_WRITE = 1,   // write=1 track=0
  TRACK_PLAY    = 2,   // write=0 track=1
  TRACK_WRITE   = 3,   // write=1 track=1
};

// TRACK·INSTRUMENT rotary order (== the 3-bit code)
enum Instrument : uint8_t {
  INST_ACCENT = 0, INST_BD, INST_SD, INST_LT, INST_HT, INST_CY, INST_OH, INST_CH,
  NUM_INSTRUMENTS
};

struct Controls {
  uint8_t cell[4] = {0,0,0,0};  // per select: bit0-3 = PB0-3, bit4-7 = PA0-3
  uint8_t status_hi = 0;        // PA0-3 with all selects high

  void Scan() { hw::ScanMatrix(cell, &status_hi); }

  // raw cell access
  bool pb(uint8_t sel, uint8_t b) const { return (cell[sel] >> b) & 1; }
  bool pa(uint8_t sel, uint8_t b) const { return (cell[sel] >> (4 + b)) & 1; }

  // step switches (0..15)
  bool step(uint8_t n) const { return pb(n >> 2, n & 3); }

  // rotaries
  bool write_mode() const { return pa(0, 0); }
  bool track_mode() const { return pa(0, 1); }
  Mode mode() const { return Mode((pa(0, 1) << 1) | pa(0, 0)); }
  uint8_t scale() const { return (pa(1, 1) << 1) | pa(1, 0); }          // 0..3 (= scale 1..4)
  Instrument instrument() const { return Instrument((pa(2, 2) << 2) | (pa(2, 1) << 1) | pa(2, 0)); }

  // buttons
  bool clear() const    { return pa(3, 0); }
  bool function() const { return pa(3, 1); }
  bool group() const    { return pa(3, 2); }
  bool write_tap() const { return (status_hi >> 1) & 1; }
};

// step-LED helper: light step LED n (0..15) on the PG x PH grid
namespace hw {
  inline void LightStep(uint8_t n) { LightCell(n >> 2, n & 3); }
}
