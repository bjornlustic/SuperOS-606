// SuperOS-606 — stock TR-606 sequencer (power-on firmware)
//
// Implements the original 606 functions on the 4-position MODE dial:
//
// PATTERN WRITE
//   - The INSTRUMENT dial picks which instrument's steps are shown/edited
//     (position 0 = the global ACCENT track).
//   - RUNNING: step buttons toggle steps for that instrument; the chase light
//     runs through the pattern (XOR over the programmed steps).
//   - RUNNING + PATTERN CLEAR held: steps are deleted as the chase passes them.
//   - RUNNING + WRITE/NEXT (TAP): records the current instrument on whichever
//     step is playing at that moment.
//   - FUNCTION + step button: sets the pattern length (running or stopped).
//   - Move the SCALE switch, then press FUNCTION: the new scale is armed and
//     activates at the next start of the pattern (immediately when stopped).
//   - STOPPED: step buttons select the pattern to edit (within the current
//     group); hold a pattern's step and press PATTERN CLEAR to erase it.
//   - PATTERN GROUP toggles group I/II here too (selection + display).
//
// PATTERN PLAY  (no editing here)
//   - Step buttons select a pattern. Hold one and press another to chain the
//     range between them. While running, selections/chains queue and take over
//     when the playing pattern — or the whole active chain — finishes.
//   - PATTERN GROUP toggles between group I (patterns 1-16) and II (17-32).
//
// TRACK PLAY / TRACK WRITE  (INSTRUMENT dial 1-8 selects the track)
//   - Track write: hold a step button and press WRITE/NEXT to append that
//     pattern to the track; PATTERN CLEAR + WRITE/NEXT removes the last entry.
//   - Track play: RUN plays the track's pattern chain in a loop.
//
// Transport + timing come from the 606's own circuits, read on the PA status
// lines: START/STOP is the panel toggle flip-flop, the 24 PPQN tempo clock
// follows the TEMPO knob (or DIN sync, via the rear switch). MIDI OUT mirrors
// everything: clock/start/stop plus a note per drum hit.
//
// MIDI SYNC IN: when an external MIDI clock is arriving the sequencer slaves to
// it instead — stepping off the incoming 24 PPQN clock and running/stopping on
// MIDI Start/Stop/Continue — and falls back to the internal clock when it goes
// away. The panel START/STOP and a DAW's transport are OR'd, so either can run
// it; the received clock is forwarded to MIDI OUT for downstream gear. It is
// auto-engaged (no spare panel control): any incoming clock takes over, so
// unplug MIDI to play to the TEMPO knob / DIN sync again.
//
// Patterns and tracks persist to internal flash (see flash_persist.h) when the
// sequencer stops — if the SPM flash service is installed (service-install.syx).
//
// MIDI IN also carries the web pattern editor's SysEx link (tools/web-pattern-
// edit; protocol in midi_api.h): pattern/track dumps and pushes, with pushes
// landing in RAM immediately and persisting on the next stop / idle save.

#include <Arduino.h>
#include "pins.h"
#include "hw.h"
#include "controls.h"
#include "pattern.h"
#include "engine.h"
#include "flash_persist.h"
#include "midi_api.h"

// ---------------------------------------------------------------------------
// MIDI out
// ---------------------------------------------------------------------------
// Notes go through midi.cpp's TX queue so they can never split a SysEx pattern
// dump in two. Realtime bytes write directly: the MIDI spec lets them
// interleave anywhere, and clock must not wait behind a queued dump.
static inline void midiNoteOn(uint8_t n, uint8_t v) { const uint8_t m[3] = { 0x90, (uint8_t)(n & 0x7F), (uint8_t)(v & 0x7F) }; midi_tx_msg(m, 3); }
static inline void midiNoteOff(uint8_t n)           { const uint8_t m[3] = { 0x80, (uint8_t)(n & 0x7F), 0 }; midi_tx_msg(m, 3); }
static inline void midiRT(uint8_t b)                { Serial1.write(b); }

// ---------------------------------------------------------------------------
// Debounced inputs (3-sample shift register, sampled once per ~1 ms loop pass)
// ---------------------------------------------------------------------------
struct PinState {
  uint8_t state = 0;
  void push(bool high) { state = (uint8_t)((state << 1) | (high ? 1 : 0)); }
  bool rising()  const { return (state & 0x07) == 0x03; }
  bool falling() const { return (state & 0x07) == 0x04; }
  bool held()    const { return (state & 0x07) != 0; }
};

// Rotary debounce: accept a new position after 3 identical consecutive reads.
struct RotaryDb {
  uint8_t value = 0, cand = 0, cnt = 0;
  void update(uint8_t v) {
    if (v == value) { cnt = 0; return; }
    if (v == cand) { if (++cnt >= 3) { value = v; cnt = 0; } }
    else { cand = v; cnt = 1; }
  }
};

static Engine   eng;
static Controls panel;

static PinState stepB[MAX_STEPS];
static PinState clearB, fnB, groupB, tapB, runB, clkB;
static RotaryDb modeDb, instDb, scaleDb;

static uint8_t  disp_group = 0;       // 0 = group I, 1 = group II
static int8_t   anchor     = -1;      // pattern-play chain anchor (step index)
static Mode     prev_mode  = PATTERN_PLAY;

static uint8_t  prev_fired = 0;       // last step's voices, for MIDI note-offs

static uint16_t frame      = 0;       // step-LED frame shown by ScanAndDisplay

// --- tempo tracker for the stopped blink -----------------------------------
// While STOPPED the tempo clock on the PA3 status line is a NARROW pulse train
// straight from the oscillator (the run flip-flop IC2a/b only stretches it into
// a square wave while running), far too narrow for the ~1 ms polled scan to
// catch. A pin-change interrupt on the status pin (Teensy 23 = AVR PB3) grabs
// the pulses whenever the scan selects are idle-high and keeps a smoothed
// estimate of the 24 PPQN period; the stopped blink free-runs on that estimate,
// so it tracks the TEMPO knob live and survives any pulses the scan windows hide.
// The run/stop flip-flops DIVIDE the oscillator as well as reshape it, so the
// stopped pulse rate is a multiple of the musical 24 PPQN rate the CPU sees
// while running (hardware-observed: the naive stopped blink ran at the wrong
// tempo). The running clock is the trusted musical reference — the sequencer
// provably plays at the right tempo from it — so the tracker keeps two period
// estimates (running / stopped), measures their ratio at each stop, and
// advances the blink phase in HALF-ticks of the musical clock per stopped
// pulse. The blink then always comes out in musical quarter notes no matter
// what the divider does, and still follows the TEMPO knob while stopped.
static volatile uint32_t s_clk_last_us = 0;
static volatile uint32_t s_per_run     = 20833; // 24-PPQN period (120 BPM default)
static volatile uint32_t s_per_stop    = 0;     // stopped-pulse period
static volatile uint8_t  s_half_tick   = 0;     // musical phase in half-ticks, mod 48
static volatile uint8_t  s_k_half      = 0;     // half-ticks per stopped pulse (latched)
static volatile uint8_t  s_stop_seen   = 0;     // accepted pulses since stopping
static volatile bool     s_clk_running = false; // transport state, for the tracker

static inline void half_tick_advance(uint16_t h) {
  s_half_tick = (uint8_t)((s_half_tick + h) % 48);
}

// Shared by the ISR and the polled path (call with interrupts off).
static void clk_track_edge(uint32_t now) {
  const uint32_t d = now - s_clk_last_us;

  if (s_clk_running) {
    // wide 24-PPQN square wave: learn the musical reference period
    const uint32_t est = s_per_run;
    if (d < est - (est >> 2)) return;       // duplicate sighting of one pulse
    s_clk_last_us = now;
    uint8_t n = (uint8_t)((d + (est >> 1)) / est);
    if (n < 1) n = 1;
    if (n > 24) n = 24;
    const uint32_t d1 = d / n;
    if (d1 >= 5000UL && d1 <= 150000UL)
      s_per_run = (est * 3 + d1) >> 2;
    half_tick_advance((uint16_t)(2 * n));
    return;
  }

  // stopped: narrow pulses at some multiple of the musical rate
  const uint32_t est = s_per_stop;
  const uint32_t min_gap = est ? est - (est >> 2) : 2500UL;
  if (d < min_gap) return;
  s_clk_last_us = now;

  uint8_t n = 1;
  if (!est) {
    if (d >= 2500UL && d <= 150000UL) s_per_stop = d;     // first seed
  } else {
    n = (uint8_t)((d + (est >> 1)) / est);
    if (n < 1) n = 1;
    if (n > 24) n = 24;
    const uint32_t d1 = d / n;
    if (d1 >= 2500UL && d1 <= 150000UL)
      s_per_stop = (est * 3 + d1) >> 2;
  }

  if (s_k_half == 0) {
    // ratio not latched yet: advance by the live ratio against the musical
    // reference, and latch once the stopped estimate has had time to settle
    uint32_t h = (2 * d + (s_per_run >> 1)) / s_per_run;
    if (h < 1) h = 1;
    if (h > 16) h = 16;
    half_tick_advance((uint16_t)h);
    if (s_per_stop && ++s_stop_seen >= 12) {
      uint32_t k = (2 * s_per_stop + (s_per_run >> 1)) / s_per_run;
      if (k < 1) k = 1;
      if (k > 8) k = 8;
      s_k_half = (uint8_t)k;
    }
  } else {
    half_tick_advance((uint16_t)(s_k_half * n));
  }
}

ISR(PCINT0_vect) {
  if ((PORTF & 0x0F) != 0x0F) return;   // matrix scan in progress: PB3 != status
  if (!(PINB & (1 << 3))) return;       // only rising edges
  clk_track_edge(micros());
}

// Stopped-blink phase: quarter notes (24 half-ticks on, 24 off) at the tempo.
static bool tempo_blink() {
  uint8_t t;
  { const uint8_t sreg = SREG; cli(); t = s_half_tick; SREG = sreg; }
  return t < 24;
}

// --- external MIDI clock sync ----------------------------------------------
// When an external MIDI clock is arriving the sequencer follows it (tempo +
// transport) instead of the 606's own tempo oscillator, falling back to the
// internal clock MCLK_TIMEOUT_MS after the last pulse. Auto-detected: there is
// no spare panel control, so any incoming clock takes over — unplug MIDI to use
// the TEMPO knob / DIN sync again. s_hw_run latches the panel START/STOP
// flip-flop so it can be OR'd with the MIDI transport into one run state.
static const uint16_t MCLK_TIMEOUT_MS = 300;
static uint32_t s_last_mclk_ms = 0;
static bool     s_hw_run       = false;   // panel START/STOP toggle-FF latch
static bool     s_want_run     = false;   // combined transport (panel OR MIDI)

static inline uint8_t abs_pat(uint8_t s) { return (uint8_t)(disp_group * 16 + s); }
static inline uint16_t led_bit(uint8_t n) { return (uint16_t)1 << n; }

static void send_note_offs() {
  for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
    if (prev_fired & (1 << i)) midiNoteOff(INSTRUMENT_NOTE[i]);
  prev_fired = 0;
}

void setup() {
  // Pull up the MIDI RX line so an unplugged 3.5mm/DIN jack can't leave the
  // input floating and self-clock the UART from matrix/EMI noise — which now
  // matters because spurious 0xF8 bytes would drive the sequencer (sync IN).
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  hw::Init();
  eng.Init();

  flash_persist_begin();
  load_all(eng);   // patterns + tracks only; power-on is always pattern 1, group I

  // pin-change interrupt on the PA3 status input (tempo-clock pulse catcher)
  PCMSK0 |= _BV(PCINT3);
  PCICR  |= _BV(PCIE0);

  delay(150);
  // boot signature: three quick CC#119 pulses = sequencer firmware alive
  for (uint8_t i = 0; i < 3; ++i) {
    Serial1.write(0xB0); Serial1.write((uint8_t)0x77); Serial1.write((uint8_t)0x7F); delay(60);
    Serial1.write(0xB0); Serial1.write((uint8_t)0x77); Serial1.write((uint8_t)0x00); delay(60);
  }
}

// ---------------------------------------------------------------------------
// Mode handlers (called once per loop pass, after transport/clock processing)
// ---------------------------------------------------------------------------
static void handle_pattern_write(uint8_t inst) {
  // PATTERN GROUP toggles group I/II here too (changes which pattern the step
  // buttons select while stopped, and the pattern-number display)
  if (groupB.rising()) disp_group ^= 1;

  // SCALE switch is armed by a FUNCTION press; activates at next pattern start
  if (fnB.rising()) {
    eng.ArmScale(scaleDb.value);
    midi_send_scale_update(eng.cur_pat, scaleDb.value);
  }

  const bool fn = fnB.held();
  for (uint8_t s = 0; s < MAX_STEPS; ++s) {
    if (!stepB[s].rising()) continue;
    if (fn) {                                               // FUNCTION + step = length
      eng.SetLength(s + 1);
      midi_send_length_update(eng.cur_pat, s + 1);
    } else if (eng.running) {                               // enter/remove steps (running only)
      eng.ToggleStep(inst, s);
      midi_send_step_update(eng.cur_pat, inst, s, (eng.cur().steps[inst] >> s) & 1);
    } else {
      eng.SelectPattern(abs_pat(s));                        // stopped: pick pattern to edit
    }
  }

  if (eng.running) {
    // chase-delete: while PATTERN CLEAR is held, erase the current instrument's
    // steps as the chase light passes them
    if (clearB.held() && eng.step_advanced && eng.step >= 0) {
      const uint8_t cs = (uint8_t)eng.step;
      if ((eng.cur().steps[inst] >> cs) & 1) {
        eng.ClearStep(inst, cs);
        midi_send_step_update(eng.cur_pat, inst, cs, false);
      }
    }
    // tap-write: record the step that is playing right now
    if (tapB.rising() && eng.step >= 0) {
      eng.SetStep(inst, (uint8_t)eng.step);
      midi_send_step_update(eng.cur_pat, inst, (uint8_t)eng.step, true);
    }
  } else {
    // hold a pattern's step button + press PATTERN CLEAR = erase that pattern
    if (clearB.rising()) {
      bool cleared = false;
      for (uint8_t s = 0; s < MAX_STEPS; ++s)
        if (stepB[s].held()) {
          eng.ClearPattern(abs_pat(s));
          midi_send_pattern_dump(abs_pat(s));   // editor refreshes from the dump
          cleared = true;
        }
      if (cleared) save_dirty(eng);
    }
  }
}

static void handle_pattern_play() {
  if (groupB.rising()) disp_group ^= 1;

  for (uint8_t s = 0; s < MAX_STEPS; ++s) {
    if (!stepB[s].rising()) continue;

    // another step already held? -> build a range chain
    int8_t other = -1;
    for (uint8_t a = 0; a < MAX_STEPS; ++a)
      if (a != s && stepB[a].held()) { other = (int8_t)a; break; }

    if (other >= 0) {
      const uint8_t lo = other < (int8_t)s ? (uint8_t)other : s;
      const uint8_t hi = other < (int8_t)s ? s : (uint8_t)other;
      uint8_t pats[CHAIN_MAX], n = 0;
      for (uint8_t p = lo; p <= hi && n < CHAIN_MAX; ++p) pats[n++] = abs_pat(p);
      eng.SelectChain(pats, n);
      anchor = other;
    } else {
      eng.SelectPattern(abs_pat(s));   // immediate when stopped, queued at wrap
      anchor = (int8_t)s;
    }
  }
  if (anchor >= 0 && !stepB[anchor].held()) anchor = -1;
}

static void handle_track_modes(Mode mode, uint8_t inst) {
  eng.cur_track = inst & 7;            // INSTRUMENT dial position = track 1-8
  if (groupB.rising()) disp_group ^= 1;

  if (mode != TRACK_WRITE) return;

  if (tapB.rising()) {
    if (clearB.held()) {               // PATTERN CLEAR + WRITE/NEXT: drop last entry
      Track &t = eng.track[eng.cur_track];
      if (t.len > 0) eng.TrackTruncate(t.len - 1);
    } else {                           // held step + WRITE/NEXT: append that pattern
      for (uint8_t s = 0; s < MAX_STEPS; ++s)
        if (stepB[s].held()) { eng.TrackAppend(abs_pat(s)); break; }
    }
  }
}

// ---------------------------------------------------------------------------
// LED frame per mode
// ---------------------------------------------------------------------------
static uint16_t build_frame(Mode mode, uint8_t inst) {
  const bool blink8 = (millis() >> 6) & 1;   // ~8 Hz (queued-selection overlay)

  switch (mode) {
    case PATTERN_WRITE: {
      if (fnB.held()) return led_bit(eng.cur().length - 1);          // length readout
      if (!eng.running) {                            // stopped: pattern number, blinking
        if (eng.cur_pat / 16 != disp_group) return 0;        // at the tempo (stock-style)
        return tempo_blink() ? led_bit(eng.cur_pat % 16) : 0;
      }
      uint16_t f = eng.cur().steps[inst];                            // running: steps + chase
      if (eng.step >= 0) f ^= led_bit((uint8_t)eng.step);
      return f;
    }
    case PATTERN_PLAY: {
      uint16_t f = 0;
      for (uint8_t i = 0; i < eng.chain_len; ++i)
        if (eng.chain[i] / 16 == disp_group) f |= led_bit(eng.chain[i] % 16);
      if (eng.cur_pat / 16 == disp_group) {
        const uint16_t b = led_bit(eng.cur_pat % 16);
        // quarter-note blink both ways: beat-locked while playing (follows the
        // TEMPO knob), tracked tempo while stopped
        const bool on = eng.running ? eng.BeatBlink() : tempo_blink();
        if (on) f |= b; else f &= ~b;
      }
      for (uint8_t i = 0; i < eng.queue_len; ++i)                // queued = fast blink
        if (eng.queued[i] / 16 == disp_group && blink8) f |= led_bit(eng.queued[i] % 16);
      return f;
    }
    case TRACK_PLAY: {
      if (!eng.running)                                          // selected track, tempo blink
        return tempo_blink() ? led_bit(eng.cur_track) : 0;
      return eng.BeatBlink() ? led_bit(eng.cur_pat % 16) : 0;    // playing pattern, beat blink
    }
    case TRACK_WRITE: {
      const Track &t = eng.track[eng.cur_track];
      if (fnB.held()) return t.len ? led_bit((uint8_t)((t.len - 1) & 15)) : 0;  // count
      if (eng.running) return led_bit(eng.cur_pat % 16);
      if (!t.len) return 0;
      return tempo_blink() ? led_bit(t.pat[t.len - 1] % 16) : 0;  // last entry, tempo blink
    }
  }
  return 0;
}

void loop() {
  // 1. one combined matrix-scan + LED-display pass (~1 ms; the loop heartbeat).
  // The clock-pulse interrupt is masked during the pass: the status line hops
  // levels as the selects toggle, and those fake edges would poison the tempo
  // estimate (seen on hardware as a way-too-fast stopped blink).
  PCMSK0 &= (uint8_t)~_BV(PCINT3);
  hw::ScanAndDisplay(frame, panel.cell, &panel.status_hi);
  delayMicroseconds(8);                 // let the status gate settle after the
                                        // selects restore (its delayed rise was
                                        // landing after the flag clear below)
  PCIFR  = _BV(PCIF0);                  // drop any edge noise from the scan itself
  PCMSK0 |= _BV(PCINT3);

  // 2. debounce everything
  for (uint8_t s = 0; s < MAX_STEPS; ++s) stepB[s].push(panel.step(s));
  clearB.push(panel.clear());
  fnB.push(panel.function());
  groupB.push(panel.group());
  tapB.push(panel.write_tap());
  runB.push(panel.run());
  clkB.push(panel.tempo_clk());
  modeDb.update((uint8_t)panel.mode());
  instDb.update((uint8_t)panel.instrument());
  scaleDb.update(panel.scale());

  const Mode    mode = (Mode)modeDb.value;
  const uint8_t inst = instDb.value;

  // 3. engine housekeeping: end a pending trigger pulse, clear step_advanced
  eng.Service();

  // 3b. drain MIDI IN early so an external clock/transport can drive this pass.
  // mc carries the realtime clock state; received clock is already forwarded to
  // MIDI OUT inside midi_rx_poll. SysEx/notes/program-change are handled here
  // too (remote selections move disp_group, which sections 6/8 then display).
  MidiClockIn mc;
  midi_rx_poll(eng, disp_group, mc);
  if (mc.pulses) s_last_mclk_ms = millis();
  // uint32_t (not uint16_t) so a long gap with no clock can't alias back under
  // the window every ~65 s and briefly suppress the internal clock.
  const bool ext_sync = (uint32_t)(millis() - s_last_mclk_ms) < MCLK_TIMEOUT_MS;

  // 4. transport — the panel START/STOP toggle-FF OR an external MIDI transport
  // (DAW Start/Stop). Either source runs the sequencer; while a MIDI transport
  // is active the OR keeps it running, so the panel toggle is harmlessly
  // overridden. A fresh MIDI Start/Continue resyncs the pattern to the top even
  // mid-run (the 606 has no pause, so Continue restarts like Start).
  if (runB.rising())  s_hw_run = true;
  if (runB.falling()) s_hw_run = false;
  const bool want_run = s_hw_run || mc.transport;

  if ((want_run && !s_want_run) || (mc.started && want_run)) {       // start / resync
    eng.Start(mode == TRACK_PLAY || mode == TRACK_WRITE);
    { const uint8_t sreg = SREG; cli(); s_clk_running = true; SREG = sreg; }
    midiRT(0xFA);
  }
  if (!want_run && s_want_run) {                                     // stop
    eng.Stop();
    send_note_offs();
    midiRT(0xFC);
    save_dirty(eng);
    // re-measure the stopped-pulse rate and its ratio to the musical clock
    { const uint8_t sreg = SREG; cli();
      s_clk_running = false; s_per_stop = 0; s_stop_seen = 0; s_k_half = 0;
      SREG = sreg; }
  }
  s_want_run = want_run;

  // 5. clock — step from the external MIDI clock when one is present, otherwise
  // from the 606's own 24-PPQN tempo clock (TEMPO knob / DIN sync). External
  // pulses were already forwarded to MIDI OUT on receipt; the internal clock is
  // mirrored to OUT here, so the 606 is a free-running clock master when not
  // slaved. Multiple MIDI pulses can land in one pass (e.g. behind a SysEx
  // burst) — drain them all so the tempo never lags.
  uint8_t ticks = 0;
  if (ext_sync) {
    ticks = mc.pulses;
  } else if (clkB.rising()) {
    // feed the tempo tracker from the clean polled edges too (while running
    // the pulses are wide squares, so this path catches every one)
    { const uint8_t sreg = SREG; cli(); clk_track_edge(micros()); SREG = sreg; }
    midiRT(0xF8);
    ticks = 1;
  }
  for (uint8_t t = 0; t < ticks; ++t) {
    if (eng.ClockTick()) {
      send_note_offs();                              // close last step's notes
      for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
        if (eng.fired() & (1 << i))
          midiNoteOn(INSTRUMENT_NOTE[i], eng.fired_accent() ? 127 : 96);
      prev_fired = eng.fired();
      // pattern-start anchor for the web editor's playhead / pattern follow
      if (eng.step == 0) midi_send_step_position(eng.cur_pat);
    }
  }

  // 6. mode dispatch
  if (mode != prev_mode) {
    anchor = -1;
    if (!eng.running) save_dirty(eng);
    prev_mode = mode;
  }
  switch (mode) {
    case PATTERN_WRITE: handle_pattern_write(inst); break;
    case PATTERN_PLAY:  handle_pattern_play();      break;
    case TRACK_PLAY:
    case TRACK_WRITE:   handle_track_modes(mode, inst); break;
  }

  // 7. web-editor MIDI link, outgoing side: broadcast the selected pattern
  // (0x1E), service queued pattern/track dumps, and pump the TX queue. Incoming
  // SysEx/notes were already handled in step 3b. Pushed data persists once the
  // line goes idle while stopped (never mid-transfer — each SPM page write
  // halts the CPU and would drop incoming MIDI bytes).
  midi_tx_service(eng);
  if (midi_take_save_request(eng)) save_dirty(eng);

  // 8. PATTERN GROUP I/II indicator + next display frame (the stopped tempo
  // blink lives inside build_frame, on each mode's selected-pattern LED)
  eng.SetGroupLed(disp_group);
  frame = build_frame(mode, inst);
}
