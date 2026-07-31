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
static constexpr uint8_t PROB_SLOTS     = 64;   // (instrument, step, chance) slots
static constexpr uint8_t RATCHET_SLOTS  = 16;   // (instrument, step, count) slots

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
  // RATCHET: per-instrument per-step retrigger count, stored as a sparse slot
  // table (same shape as the probability table below; a dense 8x64 grid would
  // not fit the flash record). A slot binds (instrument, step) to a count v
  // (1..3): the voice fires v extra times inside the step, evenly spaced, for
  // 2x/3x/4x total. No slot = single hit. 16 slots per pattern. This replaces
  // the old whole-step flam: each voice on a step now ratchets independently.
  uint8_t  rstep[RATCHET_SLOTS];    // slot's step 0..63; 0xFF = slot free
  uint8_t  rval[RATCHET_SLOTS];     // slot's (instrument << 4) | count 1..3
  // PROBABILITY: per-instrument per-step play chance in 16ths, stored as a
  // sparse slot table (a dense 8x64 4-bit grid would not fit a 243-byte flash
  // record). A slot binds (instrument, step) to v: the hit fires v/16 of the
  // time (1..15). No slot = always fires. 64 slots per pattern.
  uint8_t  pstep[PROB_SLOTS];       // slot's step 0..63; 0xFF = slot free
  uint8_t  pval[PROB_SLOTS];        // slot's (instrument << 4) | chance 1..15
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
    length = NUM_STEP_BTNS; scale = 0; swing = 0;                  // default one bar
    memset(rstep, 0xFF, sizeof(rstep));                           // all ratchet slots free
    memset(rval, 0, sizeof(rval));
    memset(pstep, 0xFF, sizeof(pstep));                            // all slots free
    memset(pval, 0, sizeof(pval));
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
  // Ratchet slots. get returns the extra-hit count (0 = none = single hit,
  // 1..3 = 2x/3x/4x); set with v = 0 frees the slot. set returns false only
  // when the table is full (16 slots).
  uint8_t ratchet_get(uint8_t inst, uint8_t s) const {
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k)
      if (rstep[k] == s && (rval[k] >> 4) == inst) return (uint8_t)(rval[k] & 15);
    return 0;
  }
  bool ratchet_set(uint8_t inst, uint8_t s, uint8_t v) {
    if (v > 3) v = 3;
    uint8_t free_k = 0xFF;
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k) {
      if (rstep[k] == s && (rval[k] >> 4) == inst) {
        if (v) rval[k] = (uint8_t)((inst << 4) | v); else rstep[k] = 0xFF;
        return true;
      }
      if (rstep[k] == 0xFF && free_k == 0xFF) free_k = k;
    }
    if (!v) return true;
    if (free_k == 0xFF) return false;
    rstep[free_k] = s;
    rval[free_k]  = (uint8_t)((inst << 4) | v);
    return true;
  }
  void ratchet_clear_inst(uint8_t inst) {
    for (uint8_t k = 0; k < RATCHET_SLOTS; ++k)
      if (rstep[k] != 0xFF && (rval[k] >> 4) == inst) rstep[k] = 0xFF;
  }

  // Probability slots. get returns the chance in 16ths (0 = none = always);
  // set with v = 0 frees the slot. set returns false only when the table is
  // full (64 slots, effectively never in practice).
  uint8_t prob_get(uint8_t inst, uint8_t s) const {
    for (uint8_t k = 0; k < PROB_SLOTS; ++k)
      if (pstep[k] == s && (pval[k] >> 4) == inst) return (uint8_t)(pval[k] & 15);
    return 0;
  }
  bool prob_set(uint8_t inst, uint8_t s, uint8_t v) {
    uint8_t free_k = 0xFF;
    for (uint8_t k = 0; k < PROB_SLOTS; ++k) {
      if (pstep[k] == s && (pval[k] >> 4) == inst) {
        if (v) pval[k] = (uint8_t)((inst << 4) | v); else pstep[k] = 0xFF;
        return true;
      }
      if (pstep[k] == 0xFF && free_k == 0xFF) free_k = k;
    }
    if (!v) return true;
    if (free_k == 0xFF) return false;
    pstep[free_k] = s;
    pval[free_k]  = (uint8_t)((inst << 4) | v);
    return true;
  }
  void prob_clear_inst(uint8_t inst) {
    for (uint8_t k = 0; k < PROB_SLOTS; ++k)
      if (pstep[k] != 0xFF && (pval[k] >> 4) == inst) pstep[k] = 0xFF;
  }
};

// steps (8*4*2=64) + length + scale + swing + rstep(16) + rval(16)
// + pstep(64) + pval(64) + ilen(8) + poly = 236 (fits the block store's
// 243-byte record cap). Changing this cleanly invalidates older saves: the
// block store's read() returns the stored length, and load only deserializes
// when it equals PATTERN_BYTES, so mismatched old records simply don't load,
// no version byte needed.
static constexpr uint8_t PATTERN_BYTES =
    NUM_INSTRUMENTS * NUM_SECTIONS * 2 + 3 + RATCHET_SLOTS * 2 + PROB_SLOTS * 2 + NUM_INSTRUMENTS + 1;  // 236

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
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) buf[o++] = p.rstep[i];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) buf[o++] = p.rval[i];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) buf[o++] = p.pstep[i];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) buf[o++] = p.pval[i];
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
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) p.rstep[i] = buf[o++];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) p.rval[i]  = buf[o++];
  for (uint8_t i = 0; i < RATCHET_SLOTS; ++i) {
    // Sanitize: a used slot needs a valid step, a real instrument and a
    // non-zero count; anything else (e.g. an editor-zeroed blob) frees it.
    if (p.rstep[i] == 0xFF) continue;
    if (p.rstep[i] >= MAX_STEPS || (p.rval[i] >> 4) >= NUM_INSTRUMENTS || !(p.rval[i] & 15))
      p.rstep[i] = 0xFF;
  }
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) p.pstep[i] = buf[o++];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) p.pval[i]  = buf[o++];
  for (uint8_t i = 0; i < PROB_SLOTS; ++i) {
    // Sanitize: a used slot needs a valid step, a real instrument and a
    // non-zero chance; anything else (e.g. an editor-zeroed blob) frees it.
    if (p.pstep[i] == 0xFF) continue;
    if (p.pstep[i] >= MAX_STEPS || (p.pval[i] >> 4) >= NUM_INSTRUMENTS || !(p.pval[i] & 15))
      p.pstep[i] = 0xFF;
  }
  for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
    p.ilen[i] = buf[o++];
    if (p.ilen[i] > MAX_STEPS) p.ilen[i] = 0;   // invalid -> follow pattern length
  }
  p.poly = buf[o++];
  if (p.length < 1 || p.length > MAX_STEPS) p.length = NUM_STEP_BTNS;
  if (p.scale > 3) p.scale = 0;
  if (p.swing > 15) p.swing = 0;
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
