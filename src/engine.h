// SuperOS-606 — sequencer engine
//
// Clocked by the 606's own tempo clock (24 PPQN on the PA status lines, the
// same analog clock the stock CPU used). Run state is the START/STOP toggle
// flip-flop level on the status lines. Each step boundary fires the voices
// whose bit is set in the playing pattern: instrument-data lines high + a
// ~1.5 ms COMMON TRIG pulse on PI2, with the AC line high when the step is
// accented (the stock global accent).
//
// Pattern selection/queueing, range chains, and track (chained-pattern)
// playback live here too; main.cpp owns the buttons and calls in.
#pragma once
#include <Arduino.h>
#include "pins.h"
#include "pattern.h"

static constexpr uint16_t TRIG_PULSE_US = 1500;

// MIDI-IN drum hits are gathered before firing. A DAW sends the notes of one
// "chord" as separate Note Ons, but the 606 has a SINGLE shared common-trigger
// edge and a SINGLE shared accent bus per hit. Firing one pulse per note raced
// the voices across pulse windows and let a later note pull the shared accent
// down, so stacking voices accented unpredictably. Instead we accumulate the
// incoming voices/accent and fire ONE pulse once no new note has arrived for
// this long — every stacked voice then latches on the same edge with the same
// accent, exactly like a sequencer step.
//
// The window must exceed the gap between two of a chord's notes as the main
// loop SEES them: the RX FIFO is drained once per ~1ms pass, so a chord's notes
// arrive roughly one per pass and the effective spacing is one loop period plus
// jitter. 2ms was only ~2 passes of margin, so an occasional long pass split a
// chord into two pulses with separate accents (the "accents on some hits only"
// bug). 5ms rides out that jitter while staying far below the spacing of even
// 64th notes (>20ms), so it never merges genuinely separate hits. The cost is a
// fixed ~5ms trigger latency, which for DAW-timeline playback is just a constant
// offset, not timing jitter. Raise it further if splits persist on hardware.
static constexpr uint16_t TRIG_GATHER_US = 5000;

// ACCENT is the fourth INSTRUMENT-DATA bit: AC = µPD650 PF3 = socket pin 19 =
// Teensy 17 (= AC_PIN, see pins.h; service manual p.2). It is driven like the
// seven voice bits — raised on accented steps, then pulsed by COMMON TRIG (PI2).
// On the main board the accent generator (Q9/Q10/Q11 + the VR2 ACCENT pot) pulses
// the AC LEVEL bus, boosting whatever voices fire that step. AC idles LOW = no
// accent.
//
// NOTE: the accent line and its polarity are not yet confirmed on hardware —
// this follows the service manual's instrument-data model and needs verifying.

// PATTERN GROUP I/II indicator LEDs (D429/D430 via Q413/Q414, SM p.4) hang off
// PE0 (socket pin 12): one level lights I, the other lights II. PE0 doubles as
// the CH data bit during trigger pulses, so the firmware time-multiplexes it
// like the stock CPU did: group level at idle, CH data during the ~1.5ms pulse.
// (If the indicator turns out to live on PE1 instead, swap GROUP_IIL_PINS.)
static const uint8_t GROUP_IIL_PINS[] = { PE0_PIN, PE1_PIN };  // driven together

// Instrument-data pin per Instrument enum value (INST_ACCENT..INST_CH).
// Matches the µPD650 INSTRUMENT-DATA assignment in the service manual p.2:
// PF0-3 = LT/SD/BD/AC, PE0-3 = CH/OH/CY/HT.
static const uint8_t INSTRUMENT_PIN[NUM_INSTRUMENTS] = {
  AC_PIN,   // INST_ACCENT (AC) — µPD650 PF3, socket pin 19 (= Teensy 17)
  PF2_PIN,  // INST_BD        — µPD650 PF2, socket pin 18
  PF1_PIN,  // INST_SD        — µPD650 PF1, socket pin 17
  PF0_PIN,  // INST_LT        — µPD650 PF0, socket pin 16
  PE3_PIN,  // INST_HT        — µPD650 PE3, socket pin 15
  PE2_PIN,  // INST_CY        — µPD650 PE2, socket pin 14
  PE1_PIN,  // INST_OH        — µPD650 PE1, socket pin 13
  PE0_PIN,  // INST_CH        — µPD650 PE0, socket pin 12
};

// MIDI note per instrument (GM-ish drum map; accent has no note of its own).
static const uint8_t INSTRUMENT_NOTE[NUM_INSTRUMENTS] = {
  0, 36, 38, 45, 50, 49, 46, 42,
};

static constexpr uint8_t CHAIN_MAX = 16;

class Engine {
 public:
  Pattern pattern[NUM_PATTERNS];
  Track   track[NUM_TRACKS];

  // ---- playback state ----
  bool    running    = false;
  bool    track_play = false;   // started while the dial was in a TRACK mode
  uint8_t cur_pat    = 0;       // absolute pattern index 0..31 (playing / edit target)
  int8_t  step       = -1;      // chase position, -1 = before the first step
  uint8_t cur_track  = 0;
  uint8_t track_pos  = 0;

  // scale active right now; pattern's own scale is latched at each pattern start.
  uint8_t pending_scale = 0xFF; // SCALE switch value armed with FUNCTION, 0xFF = none

  // ---- pattern chain (PATTERN PLAY) ----
  uint8_t chain[CHAIN_MAX];     // active chain (absolute pattern indices)
  uint8_t chain_len = 0;        // 0 = no chain, plain single-pattern looping
  uint8_t chain_pos = 0;
  uint8_t queued[CHAIN_MAX];    // chain to adopt at the next pattern wrap
  uint8_t queue_len = 0;

  // ---- dirty flags for persistence ----
  uint32_t dirty_pat = 0;       // bit per pattern
  uint8_t  dirty_trk = 0;       // bit per track

  // set true for exactly one main-loop pass after each step advance
  bool step_advanced = false;

  void Init() {
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) pattern[i].Clear();
    for (uint8_t i = 0; i < NUM_TRACKS; ++i)  track[i].Clear();
    // AC (PF3) idles LOW = no accent (hw::Init also covers it via QUIESCENT_LOW_PINS;
    // set here too so the engine never depends on init order). Raised only on
    // accented steps, at trigger time.
    pinMode(AC_PIN, OUTPUT);
    digitalWrite(AC_PIN, LOW);
  }

  // Drive the PATTERN GROUP I/II indicator (PE0/PE1 idle level). Deferred to
  // the end of a trigger pulse if one is in flight (the lines carry CH/OH data
  // during the pulse, exactly like the stock CPU's multiplexing).
  void SetGroupLed(uint8_t group) {
    group_led_ = group & 1;
    if (!pulse_on_)
      for (uint8_t i = 0; i < sizeof(GROUP_IIL_PINS); ++i)
        digitalWrite(GROUP_IIL_PINS[i], group_led_ ? HIGH : LOW);
  }

  Pattern &cur() { return pattern[cur_pat]; }
  uint8_t  active_scale() const { return active_scale_; }

  // ---- transport ------------------------------------------------------------
  void Start(bool in_track_mode) {
    running    = true;
    track_play = in_track_mode;
    step       = -1;
    tick_      = 0;
    if (track_play) {
      track_pos = 0;
      if (track[cur_track].len > 0) cur_pat = track[cur_track].pat[0];
    } else {
      chain_pos = 0;
      if (chain_len > 0) cur_pat = chain[0];
    }
    latch_scale();
  }

  void Stop() {
    running   = false;
    step      = -1;
    tick_     = 0;
    queue_len = 0;   // a never-reached queued selection must not outlive the run
  }

  // Quarter-note beat indicator, phase-locked to the pattern: on for the first
  // half of each beat (24 ticks), counted from the pattern's first step.
  bool BeatBlink() const {
    if (!running || step < 0) return false;
    const uint16_t t = (uint16_t)step * SCALE_TICKS[active_scale_] + tick_;
    return (t % 24) < 12;
  }

  // One 24-PPQN tempo-clock tick. Returns true on a step boundary.
  bool ClockTick() {
    if (!running) return false;
    if (tick_ == 0) {
      advance_step();
      tick_ = 1;
      return true;
    }
    if (++tick_ >= SCALE_TICKS[active_scale_]) tick_ = 0;
    return false;
  }

  // Call every main-loop pass: ends a pending trigger pulse, clears the
  // step_advanced flag set by the previous pass.
  void Service() {
    step_advanced = false;
    if (pulse_on_ && (uint16_t)(micros() - pulse_t0_) >= TRIG_PULSE_US) {
      digitalWrite(PI2_PIN, LOW);
      for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i) digitalWrite(INSTRUMENT_PIN[i], LOW);
      digitalWrite(AC_PIN, LOW);       // accent off between steps
      pulse_on_ = false;
      SetGroupLed(group_led_);   // restore the I/II indicator level on PE0/PE1
    }
    // Fire a gathered MIDI-IN hit once the chord has settled and the line is
    // free, so all its voices latch on one common-trigger edge (see TriggerNow).
    if (gather_active_ && !pulse_on_ &&
        (uint16_t)(micros() - gather_last_) >= TRIG_GATHER_US) {
      const uint8_t m = gather_mask_;
      const bool    a = gather_accent_;
      gather_mask_   = 0;
      gather_accent_ = false;
      gather_active_ = false;
      fire_pulse(m, a);
    }
  }

  // ---- editing (main.cpp calls these; they mark dirty bits) ----------------
  void ToggleStep(uint8_t inst, uint8_t s) {
    cur().steps[inst] ^= (uint16_t)1 << s;
    mark_pat_dirty(cur_pat);
  }
  void SetStep(uint8_t inst, uint8_t s) {
    cur().steps[inst] |= (uint16_t)1 << s;
    mark_pat_dirty(cur_pat);
  }
  void ClearStep(uint8_t inst, uint8_t s) {
    cur().steps[inst] &= ~((uint16_t)1 << s);
    mark_pat_dirty(cur_pat);
  }
  void SetLength(uint8_t len) {
    if (len < 1) len = 1;
    if (len > MAX_STEPS) len = MAX_STEPS;
    cur().length = len;
    mark_pat_dirty(cur_pat);
  }
  void ClearPattern(uint8_t p) {
    pattern[p].Clear();
    mark_pat_dirty(p);
  }
  // Arm a scale change (FUNCTION press); applies at the next pattern start,
  // immediately when stopped.
  void ArmScale(uint8_t scale) {
    pending_scale = scale & 3;
    if (!running) latch_scale();
  }

  // One-shot voice trigger from MIDI IN (web-editor audition / external pads /
  // a DAW playing the drums), leaving the sequencer state alone. Bit 0 is the
  // accent track, not a voice, so it is masked off. Rather than firing now, the
  // voice and its accent join the gather buffer; Service() fires one combined
  // common-trigger pulse once the chord is complete (see TRIG_GATHER_US), so
  // every stacked note latches on the same edge with one shared accent.
  void TriggerNow(uint8_t mask, bool accent) {
    mask &= 0xFE;
    if (!mask) return;
    gather_mask_   |= mask;
    gather_accent_ |= accent;       // any accented note accents the whole hit
    gather_active_  = true;
    gather_last_    = micros();
  }

  // ---- pattern selection / chains (PATTERN PLAY) ----------------------------
  // Select a single pattern: immediate when stopped. While running it queues —
  // taking over when the playing pattern ends, or, if a chain is active, only
  // after the chain's last pattern has played out (see on_wrap).
  void SelectPattern(uint8_t p) {
    if (!running) {
      cur_pat   = p;
      chain_len = 0;
      chain_pos = 0;
      queue_len = 0;
      latch_scale();
    } else {
      queued[0] = p;
      queue_len = 1;
    }
  }
  // DAW program-change select. Behaves exactly like SelectPattern (immediate
  // when stopped, queued for the next wrap while running) EXCEPT when we are
  // still on the first step of the current pattern: then it switches NOW. A DAW
  // puts a program change on a bar line, but its bytes can reach us a loop pass
  // or two after that bar's own clock tick already wrapped the sequencer — so a
  // plain queue would miss the bar it was meant for and wait a whole pattern.
  // Snapping while step == 0 lands it on that bar. Panel / web-editor selects
  // never call this; they always wait for the wrap (unchanged feel on the box).
  void SelectPatternSynced(uint8_t p) {
    if (running && step == 0) {
      cur_pat   = p;
      chain_len = 0;
      chain_pos = 0;
      queue_len = 0;
      latch_scale();
    } else {
      SelectPattern(p);
    }
  }
  // Select a range chain [first..last] (absolute indices, same group).
  void SelectChain(const uint8_t *pats, uint8_t n) {
    if (n > CHAIN_MAX) n = CHAIN_MAX;
    if (!running) {
      memcpy(chain, pats, n);
      chain_len = n;
      chain_pos = 0;
      cur_pat   = chain[0];
      latch_scale();
    } else {
      memcpy(queued, pats, n);
      queue_len = n;
    }
  }

  // ---- track editing ---------------------------------------------------------
  bool TrackAppend(uint8_t p) {
    Track &t = track[cur_track];
    if (t.len >= TRACK_MAX_PATS) return false;
    t.pat[t.len++] = p;
    mark_trk_dirty(cur_track);
    return true;
  }
  void TrackTruncate(uint8_t new_len) {
    Track &t = track[cur_track];
    if (new_len < t.len) { t.len = new_len; mark_trk_dirty(cur_track); }
  }
  void TrackClear() {
    track[cur_track].Clear();
    mark_trk_dirty(cur_track);
  }

  void mark_pat_dirty(uint8_t p) { dirty_pat |= (uint32_t)1 << p; }
  void mark_trk_dirty(uint8_t t) { dirty_trk |= (uint8_t)1 << t; }

  // Voices fired on the most recent step (bitmask by Instrument), for MIDI out.
  uint8_t fired() const { return fired_; }
  bool fired_accent() const { return fired_accent_; }

 private:
  uint8_t  tick_         = 0;
  uint8_t  active_scale_ = 0;
  bool     pulse_on_     = false;
  uint32_t pulse_t0_     = 0;
  uint8_t  pulse_mask_   = 0;     // voices driven by the in-flight pulse (merged)
  bool     pulse_accent_ = false; // accent for the in-flight pulse (OR of all voices)
  uint8_t  gather_mask_   = 0;    // MIDI-IN voices accumulating for the next pulse
  bool     gather_accent_ = false;// accent for the gathered hit (OR of its notes)
  bool     gather_active_ = false;// a MIDI-IN hit is being gathered
  uint32_t gather_last_   = 0;    // micros() of the most recent gathered note
  uint8_t  fired_        = 0;
  bool     fired_accent_ = false;
  uint8_t  group_led_    = 0;     // PATTERN GROUP I/II indicator level (PE0/PE1 idle)

  void latch_scale() {
    if (pending_scale != 0xFF) {
      cur().scale = pending_scale;
      mark_pat_dirty(cur_pat);
      pending_scale = 0xFF;
    }
    active_scale_ = cur().scale;
  }

  void advance_step() {
    int8_t next = step + 1;
    if (next >= (int8_t)cur().length) {
      on_wrap();          // may switch cur_pat (chain/track/queue) + latch scale
      next = 0;
    }
    step = next;
    fire_step((uint8_t)next);
    step_advanced = true;
  }

  void on_wrap() {
    if (track_play) {
      const Track &t = track[cur_track];
      if (t.len > 0) {
        if (++track_pos >= t.len) track_pos = 0;
        cur_pat = t.pat[track_pos];
      }
    } else if (queue_len > 0 && (chain_len == 0 || chain_pos + 1 >= chain_len)) {
      // a queued selection takes over only once the playing pattern — or the
      // whole active chain — has finished: a pick made mid-chain waits for
      // the chain's last pattern to play out
      memcpy(chain, queued, queue_len);
      chain_len = (queue_len > 1) ? queue_len : 0;
      chain_pos = 0;
      cur_pat   = chain_len ? chain[0] : queued[0];
      queue_len = 0;
    } else if (chain_len > 0) {
      if (++chain_pos >= chain_len) chain_pos = 0;
      cur_pat = chain[chain_pos];
    }
    latch_scale();
  }

  void fire_step(uint8_t s) {
    const uint16_t bit = (uint16_t)1 << s;
    const Pattern &p   = cur();

    uint8_t mask   = 0;
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      if (p.steps[i] & bit) mask |= (uint8_t)1 << i;
    const bool accent = (p.steps[INST_ACCENT] & bit) != 0;

    fired_        = mask;
    fired_accent_ = accent;
    if (!mask && !accent) return;
    fire_pulse(mask, accent);
  }

  // Drive every data line to its value (PE0/PE1 may idle HIGH for the group
  // LED, so non-firing voices must be forced LOW, not left alone), AC raised
  // only on accented hits, then the common-trigger pulse. Shared by the
  // sequencer (fire_step) and one-shot MIDI-IN triggers (TriggerNow).
  //
  // Accent is a SINGLE shared bus (the AC LEVEL bus, see top of file): it boosts
  // every voice firing in the trigger window, so the box has one accent per hit,
  // not one per voice. A DAW chord arrives as separate MIDI Note Ons ~0.7ms
  // apart, landing inside one pulse window. If we restarted the pulse per note,
  // an un-accented note arriving after an accented one would pull the shared
  // accent line (and the earlier note's data line) back down — so the hit ended
  // up un-accented or partly silenced depending on note order. Instead, merge
  // into the live pulse: OR the new voice into the data lines and keep accent
  // HIGH if ANY voice in the window is accented. Keep the original pulse_t0_ so
  // a chord can't stretch the pulse — the merged voices land microseconds in,
  // with the full ~1.5ms window still ahead of them.
  void fire_pulse(uint8_t mask, bool accent) {
    if (pulse_on_) {
      pulse_mask_   |= mask;
      pulse_accent_ |= accent;
    } else {
      pulse_mask_   = mask;
      pulse_accent_ = accent;
      pulse_t0_     = micros();
      pulse_on_     = true;
    }
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      digitalWrite(INSTRUMENT_PIN[i], (pulse_mask_ & (1 << i)) ? HIGH : LOW);
    digitalWrite(AC_PIN, pulse_accent_ ? HIGH : LOW);
    digitalWrite(PI2_PIN, HIGH);
  }
};
