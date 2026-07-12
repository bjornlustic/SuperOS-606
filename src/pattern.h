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

static constexpr uint8_t NUM_STEP_BTNS  = 16;   // physical step buttons / LEDs (one section)
static constexpr uint8_t NUM_SECTIONS   = 4;    // 16-step sections per pattern
static constexpr uint8_t MAX_STEPS      = NUM_STEP_BTNS * NUM_SECTIONS;  // 64 steps max
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

// A pattern is up to 64 steps = 4 sections of 16, stored as one 16-bit word per
// instrument per section. Step index s (0..63): section = s >> 4, bit = s & 15.
struct Pattern {
  uint16_t steps[NUM_INSTRUMENTS][NUM_SECTIONS];  // [inst][section], bit = step
  uint8_t  length;                  // 1..64
  uint8_t  scale;                   // 0..3 (SCALE switch value at write time)
  uint8_t  swing;                   // 0 = off .. 15 = max (delays off-beat steps)
  uint8_t  flam_type;               // 0 = off, 1..3 = number of extra sub-hits
  uint16_t flam[NUM_SECTIONS];      // bit = step retriggers (flam/roll/ratchet)
  uint8_t  prob[MAX_STEPS];         // per-step play chance level: 0 = always, 1..3 = rarer
  uint8_t  ilen[NUM_INSTRUMENTS];   // per-instrument loop length: 0 = follow the
                                    // pattern length; 1..64 = this row loops its
                                    // own length (mode per the poly bit below)
  uint8_t  poly;                    // bit per instrument: 0 = the row RESTARTS at
                                    // every bar start (default, stays on the beat);
                                    // 1 = free-running POLYMETER (the row's loop
                                    // carries across bars). Only matters when the
                                    // row has its own ilen.

  void Clear() {
    memset(steps, 0, sizeof(steps));
    length = NUM_STEP_BTNS; scale = 0; swing = 0; flam_type = 0;   // default one bar
    memset(flam, 0, sizeof(flam));
    memset(prob, 0, sizeof(prob));
    memset(ilen, 0, sizeof(ilen));
    poly = 0;                                                      // default bar-reset
  }
  bool Empty() const {
    for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
      for (uint8_t s = 0; s < NUM_SECTIONS; ++s) if (steps[i][s]) return false;
    return true;
  }

  // Absolute-step accessors (s = 0..MAX_STEPS-1).
  bool step_get(uint8_t inst, uint8_t s) const { return (steps[inst][s >> 4] >> (s & 15)) & 1; }
  void step_set(uint8_t inst, uint8_t s) { steps[inst][s >> 4] |=  (uint16_t)1 << (s & 15); }
  void step_clr(uint8_t inst, uint8_t s) { steps[inst][s >> 4] &= ~((uint16_t)1 << (s & 15)); }
  void step_tog(uint8_t inst, uint8_t s) { steps[inst][s >> 4] ^=  (uint16_t)1 << (s & 15); }
  bool flam_get(uint8_t s) const { return (flam[s >> 4] >> (s & 15)) & 1; }
  void flam_tog(uint8_t s) { flam[s >> 4] ^= (uint16_t)1 << (s & 15); }
};

// steps (8*4*2=64) + length + scale + swing + flam_type + flam (4*2=8) + prob(64)
// + ilen(8) + poly = 149. Growing this cleanly invalidates older saves: the
// block store's read() returns the stored length, and load only deserializes
// when it equals PATTERN_BYTES, so shorter old records simply don't load — no
// version byte needed.
static constexpr uint8_t PATTERN_BYTES =
    NUM_INSTRUMENTS * NUM_SECTIONS * 2 + 4 + NUM_SECTIONS * 2 + MAX_STEPS + NUM_INSTRUMENTS + 1;  // 149

inline void serialize_pattern(const Pattern &p, uint8_t *buf) {
  uint8_t o = 0;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      buf[o++] = (uint8_t)(p.steps[i][s] & 0xFF);
      buf[o++] = (uint8_t)(p.steps[i][s] >> 8);
    }
  buf[o++] = p.length;
  buf[o++] = p.scale;
  buf[o++] = p.swing;
  buf[o++] = p.flam_type;
  for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
    buf[o++] = (uint8_t)(p.flam[s] & 0xFF);
    buf[o++] = (uint8_t)(p.flam[s] >> 8);
  }
  for (uint8_t i = 0; i < MAX_STEPS; ++i) buf[o++] = p.prob[i];
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) buf[o++] = p.ilen[i];
  buf[o++] = p.poly;
}

inline void deserialize_pattern(Pattern &p, const uint8_t *buf) {
  uint8_t o = 0;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
    for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
      p.steps[i][s] = (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
      o += 2;
    }
  p.length    = buf[o++];
  p.scale     = buf[o++];
  p.swing     = buf[o++];
  p.flam_type = buf[o++];
  for (uint8_t s = 0; s < NUM_SECTIONS; ++s) {
    p.flam[s] = (uint16_t)buf[o] | ((uint16_t)buf[o + 1] << 8);
    o += 2;
  }
  for (uint8_t i = 0; i < MAX_STEPS; ++i) p.prob[i] = buf[o++] & 3;
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
    p.ilen[i] = buf[o++];
    if (p.ilen[i] > MAX_STEPS) p.ilen[i] = 0;   // invalid -> follow pattern length
  }
  p.poly = buf[o++];
  if (p.length < 1 || p.length > MAX_STEPS) p.length = NUM_STEP_BTNS;
  if (p.scale > 3) p.scale = 0;
  if (p.swing > 15) p.swing = 0;
  if (p.flam_type > 3) p.flam_type = 0;
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
