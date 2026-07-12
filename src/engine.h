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

// Play direction (DIRECTION tool, step buttons 1-4). Runtime only, not persisted
// (matches the 6oh6, where direction is a live setting).
enum PlayDir : uint8_t { DIR_FWD = 0, DIR_BCK = 1, DIR_RND = 2, DIR_PP = 3 };

// Pattern storage is FLASH-RESIDENT (SuperOS-303 model): only the active pattern
// lives in RAM (the cache), so patterns can grow without multiplying RAM by 32.
// These hooks read/write one pattern to the flash block store; registered at boot
// via SetPatIO (see flash_persist.h). Reads are cheap + non-blocking; a flash
// write HALTS the CPU for milliseconds, so the engine only writes while stopped
// (or defers to the next stop for edits made mid-play).
using PatReadFn  = bool (*)(uint8_t idx, Pattern &out);
using PatWriteFn = bool (*)(uint8_t idx, const Pattern &in);

class Engine {
 public:
  Track   track[NUM_TRACKS];   // tracks stay RAM-resident (small)

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
  uint8_t  dirty_trk = 0;       // bit per track (patterns use the cache dirty flag)

  // set true for exactly one main-loop pass after each step advance
  bool step_advanced = false;

  // ---- performance state (runtime; not persisted) ----
  uint8_t play_dir   = DIR_FWD; // DIRECTION tool
  uint8_t mute_mask  = 0;       // MUTE tool: bit per Instrument, 1 = muted (bit0 = accent)
  uint8_t global_len = 0;       // LENGTH tool: 0 = use each pattern's own length,
                                //   1..16 = force this length on ALL patterns
  // (SWING and FLAM are per pattern now — Pattern::swing / flam / flam_type.)

  // RESLICE tool: while engaged, playback loops a window of the pattern.
  bool    reslice_on  = false;
  uint8_t reslice_len = 0;      // window length in steps

  // ARP tool: a live sequence that REPLACES the pattern while it has notes.
  uint8_t arp[16];              // instrument per arp step (0xFF = rest)
  uint8_t arp_len = 0;          // 0 = arp off

  // Register the flash read/write hooks (call once at boot, before playback).
  void SetPatIO(PatReadFn r, PatWriteFn w) { pat_read_ = r; pat_write_ = w; }

  void Init() {
    for (uint8_t i = 0; i < NUM_TRACKS; ++i)  track[i].Clear();
    cache_.Clear();
    cache_idx_ = 0xFF;            // nothing loaded yet; cur() lazy-loads
    copy_buf_.Clear();
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

  // The active pattern, backed by the flash cache. If cur_pat moved, persist the
  // outgoing pattern — to flash when stopped, or to the RAM pend slot while
  // running (a flash write halts the CPU for ms and would slip the clock). A
  // pattern parked in the pend slot is reloaded from there, never from stale
  // flash, so live edits survive chain/track wraps until the stop-time Flush.
  Pattern &cur() {
    if (cache_idx_ != cur_pat) {
      if (!running) {
        flush_();
      } else if (cache_dirty_ && cache_idx_ != 0xFF) {
        // evict dirty cache to the pend slot (live panel edits win over any
        // older deferred write parked there for a different pattern)
        pend_pat_   = cache_;
        pend_idx_   = cache_idx_;
        pend_valid_ = true;
      }
      cache_idx_   = cur_pat;
      cache_dirty_ = false;
      if (pend_valid_ && pend_idx_ == cache_idx_) {
        cache_ = pend_pat_;              // newest copy lives in the pend slot
        pend_valid_ = false;
        cache_dirty_ = true;             // still needs its flash write on stop
      } else if (!pat_read_ || !pat_read_(cache_idx_, cache_)) {
        cache_.Clear();
      }
    }
    return cache_;
  }

  // Read/write an arbitrary pattern (web editor dump/push, clear-all). Reads
  // check the cache and the pend slot before flash so they always see the newest
  // data. A write to a non-cached pattern goes straight to flash when stopped,
  // or is deferred in the pend slot when running (flash writes halt the CPU).
  void ReadPattern(uint8_t idx, Pattern &out) {
    if (idx == cache_idx_)              { out = cache_;    return; }
    if (pend_valid_ && idx == pend_idx_){ out = pend_pat_; return; }
    if (!pat_read_ || !pat_read_(idx, out)) out.Clear();
  }
  void WritePattern(uint8_t idx, const Pattern &in) {
    if (idx == cache_idx_) { cache_ = in; cache_dirty_ = true; return; }
    if (!running && pat_write_) { pat_write_(idx, in); return; }
    pend_pat_ = in; pend_idx_ = idx; pend_valid_ = true;   // defer until stopped
  }

  // Persist the dirty cache + any deferred write. Call only while stopped.
  void Flush() {
    flush_();
    if (pend_valid_) {
      if (pat_write_ && pend_idx_ != cache_idx_) pat_write_(pend_idx_, pend_pat_);
      pend_valid_ = false;               // same-idx case: cache was newer, already flushed
    }
  }

  uint8_t  active_scale() const { return active_scale_; }

  // ---- transport ------------------------------------------------------------
  void Start(bool in_track_mode) {
    running    = true;
    track_play = in_track_mode;
    step       = -1;
    tick_      = 0;
    pos_       = 0;
    first_     = true;
    pp_back_   = false;
    reslice_on = false;
    flam_rem_  = 0;
    arp_pos_   = 0;
    memset(poly_pos_, 0, sizeof(poly_pos_));  // polymeter rows start aligned
    rng_      ^= (uint32_t)micros() | 1u;   // stir the random-direction generator
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
    running    = false;
    step       = -1;
    tick_      = 0;
    queue_len  = 0;   // a never-reached queued selection must not outlive the run
    reslice_on = false;
    flam_rem_  = 0;
    arp_len    = 0;   // the arp must not outlive the run either
    arp_pos_   = 0;
  }

  // Quarter-note beat indicator, phase-locked to the pattern: on for the first
  // half of each beat (24 ticks), counted from the pattern's first step.
  bool BeatBlink() const {
    if (!running || step < 0) return false;
    // Phase off the position counter (not the played step) so the beat blink
    // stays steady in reverse / ping-pong / random directions.
    const uint16_t t = (uint16_t)pos_ * SCALE_TICKS[active_scale_] + tick_;
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
    // FLAM: retrigger the step's voices at sub-step tick offsets (tempo-locked).
    if (flam_rem_ && tick_ == flam_next_tick_) {
      fire_pulse(flam_mask_, flam_accent_);
      flam_rem_--;
      flam_next_tick_ = (uint8_t)(flam_next_tick_ + flam_spacing_);
    }
    if (++tick_ >= step_len_()) tick_ = 0;
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
  // s is an ABSOLUTE step index 0..MAX_STEPS-1 (section*16 + button).
  void ToggleStep(uint8_t inst, uint8_t s) { cur().step_tog(inst, s); mark_pat_dirty(cur_pat); }
  void SetStep(uint8_t inst, uint8_t s)    { cur().step_set(inst, s); mark_pat_dirty(cur_pat); }
  void ClearStep(uint8_t inst, uint8_t s)  { cur().step_clr(inst, s); mark_pat_dirty(cur_pat); }
  void SetLength(uint8_t len) {
    if (len < 1) len = 1;
    if (len > MAX_STEPS) len = MAX_STEPS;
    cur().length = len;
    mark_pat_dirty(cur_pat);
  }
  // Per-instrument loop length; 0 = follow the pattern length.
  void SetInstLength(uint8_t inst, uint8_t len) {
    if (len > MAX_STEPS) len = MAX_STEPS;
    cur().ilen[inst & 7] = len;
    mark_pat_dirty(cur_pat);
  }
  // Per-instrument loop mode: false = restart every bar (default), true = polymeter.
  // Switching a row resets its free-running counter, so bar-reset resyncs it to
  // the master pattern immediately and a fresh polymeter run starts aligned.
  void SetInstPoly(uint8_t inst, bool on) {
    const uint8_t bit = (uint8_t)(1 << (inst & 7));
    if (on) cur().poly |= bit; else cur().poly &= (uint8_t)~bit;
    poly_pos_[inst & 7] = 0;
    mark_pat_dirty(cur_pat);
  }
  bool GetInstPoly(uint8_t inst) { return (cur().poly >> (inst & 7)) & 1; }

  // The step this instrument is currently sounding (0..MAX_STEPS-1), or 0xFF when
  // stopped. Follow rows track the master step; bar-reset rows track step%len (so
  // they realign with the master every bar); polymeter rows track their own free-
  // running counter (which never resets on the bar).
  uint8_t inst_playhead(uint8_t inst) const {
    if (!running || step < 0) return 0xFF;
    const uint8_t li = cache_.ilen[inst & 7];
    if (!li) return (uint8_t)step;
    if ((cache_.poly >> (inst & 7)) & 1) return (uint8_t)(poly_pos_[inst & 7] % li);
    return (uint8_t)((uint8_t)step % li);
  }

  // Master mode: force EVERY row of EVERY pattern to polymeter or bar-reset,
  // clearing the individual choices. Flash-heavy (rewrites all patterns) —
  // call only while stopped.
  bool master_poly = false;   // last master choice (for the config-menu LED)
  void ApplyMasterPoly(bool poly) {
    master_poly = poly;
    const uint8_t v = poly ? 0xFF : 0x00;
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      Pattern t;
      ReadPattern(p, t);
      if (t.poly == v) continue;              // skip untouched patterns
      t.poly = v;
      WritePattern(p, t);
    }
    memset(poly_pos_, 0, sizeof(poly_pos_));   // realign every row to the bar
  }
  void ClearPattern(uint8_t p) {
    if (p == cache_idx_) { cache_.Clear(); cache_dirty_ = true; }
    else { Pattern t; t.Clear(); WritePattern(p, t); }
  }

  // COPY / PASTE tool: whole-pattern clipboard.
  void CopyPatternToBuf()    { copy_buf_ = cur(); }
  void PastePatternFromBuf() { cur() = copy_buf_; mark_pat_dirty(cur_pat); }

  // SWING tool (per pattern).
  void SetSwing(uint8_t sw) { cur().swing = sw & 15; mark_pat_dirty(cur_pat); }

  // TRANSFORM tool: operate on the current section's 16 steps for one instrument.
  void RotateInst(uint8_t inst, uint8_t sec, bool right) {
    uint16_t v = cur().steps[inst][sec];
    v = right ? (uint16_t)((v >> 1) | (v << 15)) : (uint16_t)((v << 1) | (v >> 15));
    cur().steps[inst][sec] = v;
    mark_pat_dirty(cur_pat);
  }
  void ReverseInst(uint8_t inst, uint8_t sec) {
    const uint16_t v = cur().steps[inst][sec];
    uint16_t r = 0;
    for (uint8_t i = 0; i < 16; ++i) if (v & (1u << i)) r |= (uint16_t)1 << (15 - i);
    cur().steps[inst][sec] = r;
    mark_pat_dirty(cur_pat);
  }
  // Random fill / pattern generator: density 0 (sparse) .. 3 (dense), current section.
  void RandomizeInst(uint8_t inst, uint8_t sec, uint8_t density) {
    static const uint8_t thr[4] = { 40, 90, 140, 200 };
    uint16_t v = 0;
    for (uint8_t i = 0; i < 16; ++i) if (rng_u8() < thr[density & 3]) v |= (uint16_t)1 << i;
    cur().steps[inst][sec] = v;
    mark_pat_dirty(cur_pat);
  }

  // RESLICE (live): loop a window of `win` steps from where it engaged.
  void ResliceOn(uint8_t win) {
    if (win < 1) win = 1; if (win > NUM_STEP_BTNS) win = NUM_STEP_BTNS;
    if (!reslice_on) { reslice_start_ = pos_; reslice_off_ = 0; }
    reslice_on = true; reslice_len = win;
  }
  void ResliceOff() { reslice_on = false; }

  // ARP (live): build a sequence that replaces the pattern; CLEAR empties it.
  void ArpAdd(uint8_t inst) { if (arp_len < 16) arp[arp_len++] = inst; }
  void ArpClear() { arp_len = 0; arp_pos_ = 0; }
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

  void mark_pat_dirty(uint8_t p) { if (p == cache_idx_) cache_dirty_ = true; }
  void mark_trk_dirty(uint8_t t) { dirty_trk |= (uint8_t)1 << t; }

  // Voices fired on the most recent step (bitmask by Instrument), for MIDI out.
  uint8_t fired() const { return fired_; }
  bool fired_accent() const { return fired_accent_; }

 private:
  uint8_t  tick_         = 0;
  uint8_t  pos_          = 0;     // position counter 0..length-1 (direction-agnostic)
  bool     first_        = true;  // fire the first step of a run without advancing
  bool     pp_back_      = false; // ping-pong: this cycle plays backward
  uint32_t rng_          = 0x606a55edUL;  // LCG state for random play direction
  uint8_t  active_scale_ = 0;

  uint8_t rng_u8() { rng_ = rng_ * 1664525UL + 1013904223UL; return (uint8_t)(rng_ >> 24); }

  // Effective playback length: the global override if set, else this pattern's own.
  uint8_t eff_len_() {
    if (global_len) return global_len;
    uint8_t l = cur().length;
    return l ? l : 1;
  }

  // Ticks the current step lasts. SWING lengthens on-beat (even pos_) steps and
  // shortens off-beat (odd) ones by the same amount, pushing the off-beat later
  // while keeping each pair's total — and so the tempo — constant.
  uint8_t step_len_() const {
    const uint8_t base = SCALE_TICKS[active_scale_];
    const uint8_t sw   = cache_.swing;   // cache holds the active pattern during play
    if (!sw) return base;
    uint8_t d = (uint8_t)((uint16_t)base * sw / 32);
    if (pos_ & 1) return (uint8_t)(base > d + 1 ? base - d : 2);   // off-beat: shorter
    return (uint8_t)(base + d);                                    // on-beat: longer
  }
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
  Pattern  copy_buf_;             // COPY/PASTE clipboard

  // ---- flash-resident pattern cache ----
  Pattern    cache_;              // the active pattern (== pattern[cache_idx_])
  uint8_t    cache_idx_   = 0xFF; // which pattern is cached (0xFF = none loaded)
  bool       cache_dirty_ = false;
  PatReadFn  pat_read_    = nullptr;
  PatWriteFn pat_write_   = nullptr;
  // deferred write for a non-cached pattern pushed while running (applied on stop)
  bool       pend_valid_  = false;
  uint8_t    pend_idx_    = 0;
  Pattern    pend_pat_;

  void flush_() {
    if (cache_dirty_ && pat_write_ && cache_idx_ != 0xFF) pat_write_(cache_idx_, cache_);
    cache_dirty_ = false;
  }

  // FLAM retrigger schedule (set in fire_step, serviced in ClockTick)
  uint8_t  flam_mask_    = 0;
  bool     flam_accent_  = false;
  uint8_t  flam_rem_     = 0;     // extra hits remaining this step
  uint8_t  flam_spacing_ = 0;     // ticks between sub-hits
  uint8_t  flam_next_tick_ = 0;   // tick_ value of the next sub-hit
  // RESLICE window
  uint8_t  reslice_start_ = 0;    // pos_ captured when reslice engaged
  uint8_t  reslice_off_   = 0;    // offset within the window
  // ARP playhead
  uint8_t  arp_pos_      = 0;
  // Per-row polymeter counters (free-running row positions, one per instrument)
  uint8_t  poly_pos_[NUM_INSTRUMENTS] = {0};

  void latch_scale() {
    if (pending_scale != 0xFF) {
      cur().scale = pending_scale;
      mark_pat_dirty(cur_pat);
      pending_scale = 0xFF;
    }
    active_scale_ = cur().scale;
  }

  // Advance one step. A "position counter" pos_ always counts 0..length-1 and
  // wraps once per pattern cycle (driving chain/track/queue via on_wrap, exactly
  // as before). The played step is mapped from pos_ by the play direction, so a
  // cycle is always `length` steps for chaining no matter which way it runs.
  void advance_step() {
    // 1. Musical position: ALWAYS advances with the clock and wraps every
    //    `length`. It is never disturbed by direction, reslice, arp or edits, so
    //    the bar phase — and the chain/track advance driven by on_wrap — stays
    //    locked to the beat. Whatever fancy thing we do live, releasing it drops
    //    us straight back onto the beat where the sequence would be.
    uint8_t len = eff_len_();
    if (first_) {
      first_ = false;
      pos_   = 0;
    } else if (++pos_ >= len) {
      on_wrap();          // may switch cur_pat (chain/track/queue) + latch scale
      pos_ = 0;
      if (play_dir == DIR_PP) pp_back_ = !pp_back_;   // bounce each cycle
    }
    len = eff_len_();                                 // pattern may have changed
    const uint8_t p = pos_ >= len ? (uint8_t)(len - 1) : pos_;

    // 2. Map the musical position to the sounding step by play direction.
    uint8_t s;
    switch (play_dir) {
      case DIR_BCK: s = (uint8_t)(len - 1 - p); break;
      case DIR_PP:  s = pp_back_ ? (uint8_t)(len - 1 - p) : p; break;
      case DIR_RND: s = (uint8_t)(rng_u8() % len); break;
      default:      s = p; break;
    }

    // 3. RESLICE remaps only the SOUNDING step (a live stutter); pos_ keeps
    //    running underneath, so releasing it lands back on the beat.
    if (reslice_on && reslice_len) {
      reslice_off_ = (uint8_t)((reslice_off_ + 1) % reslice_len);
      s = (uint8_t)((reslice_start_ + reslice_off_) % len);
    }

    step = (int8_t)s;
    fire_step(s);
    step_advanced = true;

    // 4. Tick the per-row polymeter counters (one tick per step, direction- and
    //    reslice-agnostic). Wrapped at each row's own loop length so a free-
    //    running row cycles cleanly; mod at use guards live ilen edits.
    {
      const Pattern &pp = cur();
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        const uint8_t li = pp.ilen[i] ? pp.ilen[i] : MAX_STEPS;
        if (++poly_pos_[i] >= li) poly_pos_[i] = 0;
      }
    }
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
    const Pattern &p   = cur();
    flam_rem_ = 0;

    uint8_t mask;
    bool    accent;

    if (arp_len) {                        // ARP replaces the pattern
      const uint8_t a = arp[arp_pos_];
      arp_pos_ = (uint8_t)((arp_pos_ + 1) % arp_len);
      mask   = (a >= INST_BD && a < NUM_INSTRUMENTS) ? (uint8_t)(1 << a) : 0;  // else = rest
      accent = false;
    } else {
      // PROBABILITY: a level > 0 gives the whole step a chance to drop out
      // (level 1 ~75% play .. level 3 ~25%). Level 0 always plays.
      if (p.prob[s]) {
        static const uint8_t thr[4] = { 255, 191, 127, 63 };
        if (rng_u8() > thr[p.prob[s] & 3]) { fired_ = 0; fired_accent_ = false; return; }
      }
      // Per-instrument loop length: a row with ilen set plays its own loop.
      // Two modes per row (Pattern::poly bit):
      //   bar-reset (default): step = s % ilen — the loop restarts at every
      //     pattern start, so every bar sounds the same and stays on the beat.
      //   polymeter: step = poly_pos_[i] % ilen — the row's own counter free-
      //     runs across bars (advanced once per step in advance_step), so the
      //     figure walks against the bar, Elektron-style.
      // ilen 0 = follow the pattern length (s passes through untouched).
      mask = 0;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i) {
        const uint8_t li = p.ilen[i];
        uint8_t si = s;
        if (li) si = (uint8_t)(((p.poly >> i) & 1 ? poly_pos_[i] : s) % li);
        if (i == INST_ACCENT) accent = p.step_get(i, si);
        else if (p.step_get(i, si)) mask |= (uint8_t)1 << i;
      }
    }

    // MUTE tool: drop muted voices (and accent, if the AC bit is muted)
    mask &= (uint8_t)~mute_mask;
    if (mute_mask & 1) accent = false;

    fired_        = mask;
    fired_accent_ = accent;

    // FLAM: schedule this hit's voices to retrigger at sub-step ticks (not in arp).
    // Spacing divides the ACTUAL step duration (step_len_ includes swing), so
    // sub-hits always land inside the step — spacing off the nominal scale ticks
    // let them spill past a swing-shortened step and vanish.
    if (!arp_len && p.flam_get(s) && p.flam_type && mask) {
      const uint8_t base = step_len_();
      uint8_t e = p.flam_type; if (e > 3) e = 3;
      uint8_t sp = (uint8_t)(base / (e + 1)); if (sp < 1) sp = 1;
      flam_mask_      = mask;
      flam_accent_    = accent;
      flam_rem_       = e;
      flam_spacing_   = sp;
      flam_next_tick_ = sp;
    }

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
