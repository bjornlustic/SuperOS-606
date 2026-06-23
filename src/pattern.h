// SuperOS-606 — pattern & track data model
//
// A pattern is 8 step-bitmaps (the 7 voices + the global ACCENT track, indexed
// by the Instrument enum / INSTRUMENT-dial order), a length (1..16) and a scale
// (the 4-position SCALE switch). 32 patterns = pattern group I (0..15) + group
// II (16..31). A track is an ordered chain of patterns, played end to end.
//
// The ACCENT track is global per step: an accented step fires the accent line
// (AC data bit) for everything triggered on that step, exactly like the stock
// 606. (Bits are one-per-step, so a future per-instrument accent — staggered
// trigger pulses — can be added without changing this layout.)
#pragma once
#include <Arduino.h>
#include <string.h>
#include "controls.h"   // Instrument enum (INST_ACCENT=0 .. INST_CH=7)

static constexpr uint8_t MAX_STEPS      = 16;
static constexpr uint8_t NUM_PATTERNS   = 32;   // group I = 0..15, group II = 16..31
static constexpr uint8_t PATS_PER_GROUP = 16;   // patterns per group (PATTERN GROUP I/II)
static constexpr uint8_t NUM_GROUPS     = NUM_PATTERNS / PATS_PER_GROUP;  // 2
static constexpr uint8_t NUM_TRACKS     = 8;    // selected with the INSTRUMENT dial
static constexpr uint8_t TRACK_MAX_PATS = 64;

// SCALE switch position (0..3 = panel "1".."4") -> tempo-clock ticks per step.
// The tempo clock runs at 24 PPQN (DIN-sync standard):
//   scale 1 = 16th notes      -> 6 ticks
//   scale 2 = 32nd notes      -> 3 ticks
//   scale 3 = 8th triplets    -> 8 ticks
//   scale 4 = 16th triplets   -> 4 ticks
static const uint8_t SCALE_TICKS[4] = { 6, 3, 8, 4 };

struct Pattern {
  uint16_t steps[NUM_INSTRUMENTS];  // bit n = step n on; index 0 = ACCENT track
  uint8_t  length;                  // 1..16
  uint8_t  scale;                   // 0..3 (SCALE switch value at write time)

  void Clear() { memset(steps, 0, sizeof(steps)); length = MAX_STEPS; scale = 0; }
  bool Empty() const {
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) if (steps[i]) return false;
    return true;
  }
};

static constexpr uint8_t PATTERN_BYTES = NUM_INSTRUMENTS * 2 + 2;  // 18

inline void serialize_pattern(const Pattern &p, uint8_t *buf) {
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
    buf[2 * i]     = (uint8_t)(p.steps[i] & 0xFF);
    buf[2 * i + 1] = (uint8_t)(p.steps[i] >> 8);
  }
  buf[16] = p.length;
  buf[17] = p.scale;
}

inline void deserialize_pattern(Pattern &p, const uint8_t *buf) {
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    p.steps[i] = (uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8);
  p.length = buf[16];
  p.scale  = buf[17];
  if (p.length < 1 || p.length > MAX_STEPS) p.length = MAX_STEPS;
  if (p.scale > 3) p.scale = 0;
}

struct Track {
  uint8_t len;                   // number of chained patterns (0 = empty track)
  uint8_t pat[TRACK_MAX_PATS];   // absolute pattern indices 0..31

  void Clear() { len = 0; memset(pat, 0, sizeof(pat)); }
};

static constexpr uint8_t TRACK_BYTES = 1 + TRACK_MAX_PATS;  // 65

inline void serialize_track(const Track &t, uint8_t *buf) {
  buf[0] = t.len;
  memcpy(buf + 1, t.pat, TRACK_MAX_PATS);
}

inline void deserialize_track(Track &t, const uint8_t *buf) {
  t.len = buf[0] > TRACK_MAX_PATS ? TRACK_MAX_PATS : buf[0];
  memcpy(t.pat, buf + 1, TRACK_MAX_PATS);
  for (uint8_t i = 0; i < TRACK_MAX_PATS; ++i)
    if (t.pat[i] >= NUM_PATTERNS) t.pat[i] = 0;
}
