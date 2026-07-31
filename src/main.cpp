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
//   - FUNCTION is the tool gateway (PATTERN WRITE only): hold FUNCTION = tool
//     map on the step LEDs, FUNCTION + step = enter that tool, tap FUNCTION =
//     exit. Tools: length (per-pattern + GROUP-chord global), scale + swing,
//     copy/paste/clear, transform, accent, probability, ratchet, mute, reslice,
//     arp, direction, pattern-gen, save. FUNCTION + CLEAR = config menu (any
//     mode). Pattern length and scale live in their tools now.
//   - 64-step patterns: 4 sections of 16. GROUP pages sections (running, or a
//     GROUP tap inside a section-aware tool); stopped, GROUP = pattern bank.
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
// MIDI SYNC IN: in the MIDI clock-source mode (Settings.clock_source, set from
// the web editor) an arriving external MIDI clock slaves the sequencer — it
// steps off the incoming 24 PPQN clock, runs/stops on MIDI Start/Stop/Continue,
// and falls back to the internal clock when it goes away; the panel START/STOP
// and a DAW's transport are OR'd, so either can run it, and the received clock
// is forwarded to MIDI OUT for downstream gear. In the INTERNAL/DIN mode the
// MIDI clock + transport are ignored and the 606 always runs from its TEMPO
// knob / rear DIN-sync jack. The MIDI receive channel and a MIDI-OUT/THRU choice
// are settings too (see settings.h / midi_api.h).
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

// Combined build (SUPEROS_COMBINED): combined.cpp owns setup()/loop() and the
// PCINT0 vector, dispatching to this firmware or the D650C emulator per the
// EEPROM firmware-select byte. Rename our entry points accordingly.
#ifdef SUPEROS_COMBINED
#include "combined.h"
#define setup superos_setup
#define loop  superos_loop
#endif
#include "settings.h"

// Global device settings (MIDI channel, clock source, OUT/THRU). Defined here,
// read from midi.cpp; loaded from flash at boot, edited live over SysEx.
Settings g_settings;

// ---------------------------------------------------------------------------
// MIDI out
// ---------------------------------------------------------------------------
// Notes go through midi.cpp's TX queue so they can never split a SysEx pattern
// dump in two. Realtime bytes write directly: the MIDI spec lets them
// interleave anywhere, and clock must not wait behind a queued dump.
//
// In THRU mode the 606's own performance output is muted — MIDI OUT instead
// mirrors MIDI IN (handled in midi.cpp) — so the note + realtime senders below
// are gated on OUT mode. The editor SysEx link (midi_tx_service) is unaffected.
static inline bool midi_out_live() { return g_settings.out_mode == OUT_MODE_OUT; }
static inline void midiNoteOn(uint8_t n, uint8_t v) { if (!midi_out_live()) return; const uint8_t m[3] = { 0x90, (uint8_t)(n & 0x7F), (uint8_t)(v & 0x7F) }; midi_tx_msg(m, 3); }
static inline void midiNoteOff(uint8_t n)           { if (!midi_out_live()) return; const uint8_t m[3] = { 0x80, (uint8_t)(n & 0x7F), 0 }; midi_tx_msg(m, 3); }
// Realtime out (clock/start/stop). Non-blocking: if the UART FIFO is momentarily
// full (e.g. a step firing a stack of voices), drop the byte rather than block —
// a stalled loop would let the RX FIFO overflow and slip the sequencer off the
// incoming clock. Downstream sync survives an occasional missed byte; the loop's
// timing must not.
static inline void midiRT(uint8_t b) {
  if (!midi_out_live()) return;
  if (Serial1.availableForWrite() > 0) Serial1.write(b);
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendRealTime(b);         // mirror clock/start/stop over USB-C
#endif
}

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

static PinState stepB[NUM_STEP_BTNS];
static PinState clearB, fnB, groupB, tapB, runB, clkB;
static RotaryDb modeDb, instDb, scaleDb;

static uint8_t  disp_group = 0;       // 0 = group I, 1 = group II
static uint8_t  disp_section = 0;     // 0..3: which 16-step section is shown/edited
static int8_t   anchor     = -1;      // pattern-play chain anchor (step index)
static Mode     prev_mode  = PATTERN_PLAY;

static uint8_t  prev_fired = 0;       // last step's voices, for MIDI note-offs

static uint16_t frame      = 0;       // step-LED frame shown by ScanAndDisplay

// FUNCTION layer: the tool selector and the config menu.
//   - Hold FUNCTION: the step LEDs show the 16-tool map.
//   - FUNCTION + step: enter that tool (stays latched).
//   - Tap FUNCTION (no step): exit the tool, back to the base mode.
//   - FUNCTION + CLEAR: open the config menu (CLEAR or FUNCTION exits).
// While a tool or the menu is active the base-mode handlers are frozen; s_menu_hold
// keeps them frozen until the exit chord is released (so the closing CLEAR can't
// chase-delete / clear a pattern), exactly as the original combined config menu did.
static uint8_t s_tool         = 0;      // 0 = base mode; 1..16 = active tool
static bool    s_fn_step_used = false;  // a step / CLEAR was used during this FN hold
static bool    s_menu         = false;  // config menu open
static bool    s_menu_hold    = false;  // exit chord still held: keep handlers frozen
static uint8_t s_scale_ref    = 0xFF;   // SCALE position at tool entry: tools that read
                                        // the switch only react to a CHANGE from this,
                                        // so entering a tool never silently applies
                                        // whatever position the switch happens to sit at
static bool    s_grp_used     = false;  // GROUP was chorded with a step inside a tool
static uint8_t s_prob_sel     = 0xFF;   // PROB tool: absolute step being edited
                                        // (flashing); 0xFF = picking a step
static uint8_t s_ratchet_sel  = 0xFF;   // RATCHET tool: absolute step being edited
                                        // (flashing); 0xFF = picking a step
static uint32_t s_fn_up_ms    = 0;      // millis() of the last FUNCTION release: step
                                        // presses shortly after are swallowed, so a
                                        // late tool-map press can't fall through and
                                        // write a stray step into the pattern
#ifdef SUPEROS_COMBINED
static constexpr uint8_t GSHARP_STEP = 8;   // step 9: reboots into the D650C emulator
#endif

// Tool IDs == step number (1..16). Only a subset is implemented in this build;
// the rest are reserved and show a placeholder on the LEDs.
enum {
  TOOL_NONE = 0,
  TOOL_LENGTH = 1, TOOL_SCALE = 2, TOOL_COPY = 3, TOOL_XFORM = 4,
  TOOL_ACCENT = 5, TOOL_PROB = 6, TOOL_RATCHET = 7, TOOL_POLY = 8,
  TOOL_MUTE = 9, TOOL_RESLICE = 10, TOOL_ARP = 11, TOOL_DIR = 12,
  TOOL_GEN = 13, TOOL_SAVE = 14,
};
// Implemented-tool slots, for the FUNCTION-held map display.
static constexpr uint16_t TOOLS_IMPLEMENTED =
  (1u << (TOOL_LENGTH - 1)) | (1u << (TOOL_SCALE   - 1)) |
  (1u << (TOOL_COPY   - 1)) | (1u << (TOOL_XFORM   - 1)) |
  (1u << (TOOL_ACCENT - 1)) | (1u << (TOOL_PROB    - 1)) |
  (1u << (TOOL_RATCHET   - 1)) | (1u << (TOOL_POLY    - 1)) |
  (1u << (TOOL_MUTE   - 1)) | (1u << (TOOL_RESLICE - 1)) |
  (1u << (TOOL_ARP    - 1)) | (1u << (TOOL_DIR     - 1)) |
  (1u << (TOOL_GEN    - 1)) | (1u << (TOOL_SAVE    - 1));

// How many 16-step sections a length spans (1..NUM_SECTIONS).
static inline uint8_t sections_for(uint8_t len) {
  if (len < 1) len = 1;
  uint8_t n = (uint8_t)(((len - 1) >> 4) + 1);
  return n > NUM_SECTIONS ? NUM_SECTIONS : n;
}
// An instrument's own reachable (playable) length: the pattern length if the row
// follows it; its own loop if polymeter; but a bar-reset row is capped by the bar
// (it plays step s%ilen with s < pattern length, so steps past the bar never sound).
static inline uint8_t inst_disp_len(uint8_t inst) {
  const Pattern &p = eng.cur();
  const uint8_t il = p.ilen[inst & 7];
  if (!il) return p.length;
  if ((p.poly >> (inst & 7)) & 1) return il;                 // polymeter: full loop
  return il < p.length ? il : p.length;                      // bar-reset: capped
}

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

#ifdef SUPEROS_COMBINED
void superos_pcint()                    // called from combined.cpp's PCINT0 ISR
#else
ISR(PCINT0_vect)
#endif
{
  if ((PORTF & 0x0F) != 0x0F) return;   // matrix scan in progress: PB3 != status
  if (!(PINB & (1 << 3))) return;       // only rising edges
  if (eng.PulseActive()) return;        // COMMON-TRIG crosstalk on the status
                                        // sense: don't let a phantom edge feed
                                        // the tempo tracker either
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

// --- DIN-sync start alignment ------------------------------------------------
// RUN (PA0) and the tempo/DIN clock (PA3) are debounced identically, but a sync
// master raises RUN on (or a hair after) a clock edge, and depending on where
// the two edges fall against the ~1 ms sampling grid the clock edge's debounced
// recognition can land one loop pass BEFORE the run's. That tick was then
// swallowed while still "stopped" and step 1 fired a whole 24-PPQN pulse
// (~20 ms at 120 BPM) late; whether it was swallowed was a sub-millisecond
// coin flip per start, so DIN sync needed repeated stop/starts to land in time.
// Remember when the last polled clock edge was seen; a start replays an edge
// this recent so step 1 stays on the master's downbeat pulse. The window is
// well under half a 24-PPQN period (>= 4 ms up to ~300 BPM), so a spec master
// that raises RUN while the clock is LOW (its NEXT edge is the downbeat) never
// trips it: its last edge is at least a half period old at RUN. In internal
// mode the stopped clock pulses are too narrow for the polled scan to see, so
// panel starts are unaffected.
static const uint32_t CLK_EDGE_GRACE_US = 3000;
static uint32_t s_clk_edge_us   = 0;      // micros() of the last polled clock edge
static bool     s_clk_edge_seen = false;  // cleared once stale / consumed

static inline uint8_t abs_pat(uint8_t s) { return (uint8_t)(disp_group * 16 + s); }
static inline uint16_t led_bit(uint8_t n) { return (uint16_t)1 << n; }

static void send_note_offs() {
  for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
    if (prev_fired & (1 << i)) midiNoteOff(INSTRUMENT_NOTE[i]);
  prev_fired = 0;
}

// USB policy, same scheme as SuperOS-303: builds made for USB MIDI
// (SUPEROS_USB_MIDI) keep USB alive and serviced; every other build tears the
// USB engine down here, because the Teensy core's usb_init() (run before
// setup()) otherwise leaves an unserviced USB controller running.
#ifndef SUPEROS_USB_MIDI
static void usb_shutdown_hw() {
  UDIEN  = 0;
  UDCON  = 1;                 // detach
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
}
#endif

void setup() {
#ifndef SUPEROS_USB_MIDI
  usb_shutdown_hw();
#endif
  // Pull up the MIDI RX line so an unplugged 3.5mm/DIN jack can't leave the
  // input floating and self-clock the UART from matrix/EMI noise — which now
  // matters because spurious 0xF8 bytes would drive the sequencer (sync IN).
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  hw::Init();
  eng.Init();

  flash_persist_begin();
  load_all(eng);          // patterns + tracks only; power-on is always pattern 1, group I
  load_settings(g_settings);   // MIDI channel / clock source / OUT-THRU (defaults if blank)

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
  // GROUP: while running it pages through the step sections that the current
  // instrument actually reaches (never past its length); while stopped it
  // toggles the pattern bank (I/II) for pattern selection.
  const uint8_t nsec = sections_for(inst_disp_len(inst));
  if (disp_section >= nsec) disp_section = 0;               // length shrank under us
  if (groupB.rising()) {
    if (eng.running) disp_section = (uint8_t)((disp_section + 1) % nsec);
    else             disp_group ^= 1;
  }

  // FUNCTION is the tool gateway (handled in loop() before dispatch), so this
  // handler only runs when FUNCTION is up. The step buttons map to the shown
  // section: absolute step = disp_section*16 + button. Steps past the current
  // instrument's length are hidden and not editable (their data is preserved).
  // A press landing just after FUNCTION was released is almost always a late
  // tool-map press, not a step entry — swallow it (see s_fn_up_ms).
  const bool fn_grace = (uint32_t)(millis() - s_fn_up_ms) < 250;
  for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) {
    if (!stepB[b].rising() || fn_grace) continue;
    if (eng.running) {                                      // enter/remove steps (running only)
      const uint8_t s = (uint8_t)(disp_section * NUM_STEP_BTNS + b);
      if (s >= inst_disp_len(inst)) continue;              // beyond this row's length
      eng.ToggleStep(inst, s);
      midi_send_step_update(eng.cur_pat, inst, s, eng.cur().step_get(inst, s));
    } else {
      eng.SelectPattern(abs_pat(b));                        // stopped: pick pattern to edit
      disp_section = 0;                                     // new pattern starts on section 1
    }
  }

  if (eng.running) {
    // chase-delete: while PATTERN CLEAR is held, erase the current instrument's
    // steps as the chase light passes them (across all sections)
    if (clearB.held() && eng.step_advanced && eng.step >= 0) {
      const uint8_t cs = (uint8_t)eng.step;
      if (eng.cur().step_get(inst, cs)) {
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
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].held()) {
          eng.ClearPattern(abs_pat(b));
          midi_send_pattern_dump(abs_pat(b));   // editor refreshes from the dump
          cleared = true;
        }
      if (cleared) save_dirty(eng);
    }
  }
}

static void handle_pattern_play() {
  if (groupB.rising()) disp_group ^= 1;

  for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s) {
    if (!stepB[s].rising()) continue;

    // another step already held? -> build a range chain
    int8_t other = -1;
    for (uint8_t a = 0; a < NUM_STEP_BTNS; ++a)
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
      for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s)
        if (stepB[s].held()) { eng.TrackAppend(abs_pat(s)); break; }
    }
  }
}

// ---------------------------------------------------------------------------
// FUNCTION tools (called instead of the base-mode handler while a tool is active)
// ---------------------------------------------------------------------------
static void handle_tool(uint8_t tool, uint8_t inst) {
  const uint8_t base = (uint8_t)(disp_section * NUM_STEP_BTNS);   // section's first step
  if (groupB.rising()) s_grp_used = false;             // fresh GROUP press: tap-vs-chord latch
  switch (tool) {
    case TOOL_LENGTH:                                  // step = pattern length (shown section)
      // The Length tool NEVER writes pattern step data — step presses here only
      // set lengths. TAP + step = the selected instrument's own loop length
      // (turn the dial to view/set each voice); GROUP + step = global length.
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) {
          const uint8_t len = (uint8_t)(base + b + 1); // absolute length 1..64
          if (tapB.held()) {
            eng.SetInstLength(inst, len);              // this voice's own loop
            midi_send_pattern_dump(eng.cur_pat);       // mirror to the web editor
          } else if (groupB.held()) {
            eng.global_len = len;                      // GROUP chord: all patterns
            s_grp_used = true;                         // don't page on release
          } else {
            eng.SetLength(len);
            midi_send_length_update(eng.cur_pat, len);
          }
        }
      if (clearB.rising()) {
        if (tapB.held()) {
          eng.SetInstLength(inst, 0);                  // TAP + CLEAR = voice follows pattern
          midi_send_pattern_dump(eng.cur_pat);         // mirror to the web editor
        } else {
          eng.global_len = 0;                          // CLEAR = clear the global override
        }
      }
      break;

    case TOOL_SCALE: {                                 // SCALE switch = scale; steps = swing
      if (scaleDb.value != s_scale_ref) {              // only on a CHANGE since tool entry
        s_scale_ref = scaleDb.value;
        eng.ArmScale(scaleDb.value);                   // applies at the next pattern wrap
        midi_send_scale_update(eng.cur_pat, scaleDb.value);
      }
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) eng.SetSwing(b);        // 0 = off .. 15 = max (per pattern)
      break;
    }

    case TOOL_ACCENT:                                  // step = toggle the accent track (this section)
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
        if (stepB[b].rising()) {
          const uint8_t s = (uint8_t)(base + b);
          eng.ToggleStep(INST_ACCENT, s);
          midi_send_step_update(eng.cur_pat, INST_ACCENT, s, eng.cur().step_get(INST_ACCENT, s));
        }
      break;

    case TOOL_PROB:
      // Two phases, INSTRUMENT dial picks the voice throughout. Pick: LEDs show
      // the voice's steps (prob'd steps blink); press a step to select it (it
      // flashes), CLEAR = wipe the voice's prob. Set: step n = the selected
      // step fires n/16 of the time (step 16 = always), CLEAR = always; either
      // returns to pick. GROUP still pages sections while picking.
      if (s_prob_sel == 0xFF) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) { s_prob_sel = (uint8_t)(base + b); break; }
        if (clearB.rising()) { eng.cur().prob_clear_inst(inst & 7); eng.mark_pat_dirty(eng.cur_pat); }
      } else {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            eng.cur().prob_set(inst & 7, s_prob_sel, (uint8_t)((b + 1) & 15));
            eng.mark_pat_dirty(eng.cur_pat);
            s_prob_sel = 0xFF;
            break;
          }
        if (clearB.rising()) {
          eng.cur().prob_set(inst & 7, s_prob_sel, 0);
          eng.mark_pat_dirty(eng.cur_pat);
          s_prob_sel = 0xFF;
        }
      }
      break;

    case TOOL_COPY:                                    // 1 clear, 2 copy, 3 paste (whole pattern)
      if (stepB[0].rising()) { eng.ClearPattern(eng.cur_pat); midi_send_pattern_dump(eng.cur_pat); }
      if (stepB[1].rising())   eng.CopyPatternToBuf();
      if (stepB[2].rising()) { eng.PastePatternFromBuf(); midi_send_pattern_dump(eng.cur_pat); }
      break;

    case TOOL_XFORM: {                                 // current instrument, shown section
      bool changed = false;
      if (stepB[0].rising()) { eng.RotateInst(inst, disp_section, false); changed = true; }  // left
      if (stepB[1].rising()) { eng.RotateInst(inst, disp_section, true);  changed = true; }  // right
      if (stepB[2].rising()) { eng.ReverseInst(inst, disp_section);       changed = true; }
      if (stepB[3].rising()) { eng.RandomizeInst(inst, disp_section, scaleDb.value & 3); changed = true; }
      if (changed) midi_send_pattern_dump(eng.cur_pat);
      break;
    }

    case TOOL_MUTE:                                    // steps 1-8 = AC..CH mute toggle
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) eng.mute_mask ^= (uint8_t)1 << b;
      if (clearB.rising()) eng.mute_mask = 0xFE;       // mute all voices (keep accent bit 0)
      if (tapB.rising())   eng.mute_mask = 0;          // unmute all
      break;

    case TOOL_DIR:                                     // steps 1-4 = fwd/back/random/ping-pong
      for (uint8_t b = 0; b < 4; ++b)
        if (stepB[b].rising()) eng.play_dir = b;
      break;

    case TOOL_GEN:                                     // TAP generates current instrument (this section)
      if (tapB.rising()) {                             // SCALE sets density
        eng.RandomizeInst(inst, disp_section, scaleDb.value & 3);
        midi_send_pattern_dump(eng.cur_pat);
      }
      break;

    case TOOL_RATCHET:
      // Two phases, INSTRUMENT dial picks the voice throughout. Pick: LEDs show
      // the voice's steps (ratcheted steps blink); press a step to select it (it
      // flashes), CLEAR = wipe the voice's ratchets. Set: step 1/2/3 = the
      // selected step retriggers 2x/3x/4x (steps 4..16 also = 4x), CLEAR =
      // single hit; either returns to pick. GROUP still pages sections.
      if (s_ratchet_sel == 0xFF) {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) { s_ratchet_sel = (uint8_t)(base + b); break; }
        if (clearB.rising()) { eng.cur().ratchet_clear_inst(inst & 7); eng.mark_pat_dirty(eng.cur_pat); }
      } else {
        for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b)
          if (stepB[b].rising()) {
            eng.cur().ratchet_set(inst & 7, s_ratchet_sel, (uint8_t)(b + 1));
            eng.mark_pat_dirty(eng.cur_pat);
            s_ratchet_sel = 0xFF;
            break;
          }
        if (clearB.rising()) {
          eng.cur().ratchet_set(inst & 7, s_ratchet_sel, 0);
          eng.mark_pat_dirty(eng.cur_pat);
          s_ratchet_sel = 0xFF;
        }
      }
      break;

    case TOOL_RESLICE: {                               // hold a step = loop that many steps
      uint8_t held = 0;
      for (uint8_t b = 0; b < NUM_STEP_BTNS; ++b) if (stepB[b].held()) { held = (uint8_t)(b + 1); break; }
      if (held) eng.ResliceOn(held); else eng.ResliceOff();
      break;
    }

    case TOOL_ARP:                                     // steps 1-8 add notes; CLEAR empties
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) eng.ArpAdd(b);          // instrument b (accent = rest)
      if (clearB.rising()) eng.ArpClear();
      break;

    case TOOL_POLY:                                    // per-instrument loop mode + master
      // steps 1-8 = AC..CH: LED lit = polymeter, off = bar-reset. Tap to toggle
      // the current pattern's row. CLEAR = master ALL rows/patterns to bar-reset,
      // TAP = master ALL to polymeter (clears the individual choices). The master
      // actions rewrite every pattern in flash, so they only fire while stopped.
      for (uint8_t b = 0; b < NUM_INSTRUMENTS; ++b)
        if (stepB[b].rising()) {
          eng.SetInstPoly(b, !eng.GetInstPoly(b));
          midi_send_pattern_dump(eng.cur_pat);         // mirror to the web editor
        }
      if (!eng.running && clearB.rising()) {
        eng.ApplyMasterPoly(false);                    // all rows, all patterns -> bar-reset
        save_dirty(eng);
        for (uint8_t p = 0; p < NUM_PATTERNS; ++p) midi_send_pattern_dump(p);
      }
      if (!eng.running && tapB.rising()) {
        eng.ApplyMasterPoly(true);                     // all rows, all patterns -> polymeter
        save_dirty(eng);
        for (uint8_t p = 0; p < NUM_PATTERNS; ++p) midi_send_pattern_dump(p);
      }
      break;

    default: break;                                    // reserved tools: no-op
  }

  // Section paging inside the section-aware tools: a GROUP tap (press + release
  // with no step chorded) advances the shown 16-step section. The LENGTH tool
  // pages the full 4 sections (you set lengths up to 64 there); the others page
  // only the sections the pattern actually reaches. GROUP + step stays a chord
  // (global length) in TOOL_LENGTH.
  switch (tool) {
    case TOOL_LENGTH: case TOOL_ACCENT: case TOOL_PROB:
    case TOOL_RATCHET:   case TOOL_XFORM:  case TOOL_GEN: {
      const uint8_t nsec = (tool == TOOL_LENGTH) ? NUM_SECTIONS
                                                 : sections_for(eng.cur().length);
      if (disp_section >= nsec) disp_section = 0;
      if (groupB.falling() && !s_grp_used)
        disp_section = (uint8_t)((disp_section + 1) % nsec);
      break;
    }
    default: break;
  }
}

// Config menu (FUNCTION + CLEAR). Panel edits to the global settings; step 10
// saves everything to flash.
static void handle_config_menu() {
  if (stepB[2].rising()) g_settings.out_mode ^= 1;        // step 3: OUT <-> THRU
  if (stepB[7].rising()) g_settings.clock_source ^= 1;    // step 8: MIDI <-> INTERNAL clock
  if (stepB[13].rising() && g_settings.midi_channel > 0)  g_settings.midi_channel--;  // step 14: ch down
  if (stepB[14].rising() && g_settings.midi_channel < 16) g_settings.midi_channel++;  // step 15: ch up
  // step 10: save all — only while stopped (each flash page write halts the CPU
  // for a few ms, which would slip the sequencer off the clock mid-play)
  if (stepB[9].rising() && !eng.running) { save_dirty(eng); save_settings(g_settings); }
  // (master polymeter/bar-reset moved to the POLY tool in the FUNCTION map)

  // Destructive clear-all (only while stopped): hold the step + press GROUP.
  if (!eng.running && groupB.rising()) {
    if (stepB[10].held()) {                                // step 11 + GROUP: clear all tracks
      for (uint8_t t = 0; t < NUM_TRACKS; ++t) { eng.track[t].Clear(); eng.mark_trk_dirty(t); }
      save_dirty(eng);
    } else if (stepB[11].held()) {                         // step 12 + GROUP: clear all patterns
      for (uint8_t p = 0; p < NUM_PATTERNS; ++p) eng.ClearPattern(p);
      save_dirty(eng);                                     // flush the cached one too
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
      if (!eng.running) {                            // stopped: pattern number, blinking
        if (eng.cur_pat / 16 != disp_group) return 0;        // at the tempo (stock-style)
        return tempo_blink() ? led_bit(eng.cur_pat % 16) : 0;
      }
      uint16_t f = eng.cur().steps[inst][disp_section];   // running: shown section's steps + chase
      // Hide steps past the current instrument's length (data is kept, just not
      // shown/editable until the length grows again).
      const uint8_t el = inst_disp_len(inst);
      const int16_t vis = (int16_t)el - (int16_t)(disp_section * NUM_STEP_BTNS);
      if (vis <= 0)                 f  = 0;
      else if (vis < NUM_STEP_BTNS) f &= (uint16_t)((1u << vis) - 1);
      // Chase = this instrument's OWN playhead (poly rows track their own loop,
      // past the bar; bar-reset/follow track the master), shown only when in the
      // viewed section.
      const uint8_t ph = eng.inst_playhead(inst);
      if (ph != 0xFF && (ph >> 4) == disp_section)
        f ^= led_bit((uint8_t)(ph & 15));
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

// True for tools whose step buttons address one 16-step section at a time.
static bool tool_uses_sections(uint8_t t) {
  return t == TOOL_LENGTH || t == TOOL_ACCENT || t == TOOL_PROB ||
         t == TOOL_RATCHET   || t == TOOL_XFORM  || t == TOOL_GEN;
}

// Section indicator on the PATTERN GROUP I/II LEDs (one selector level, so one
// LED lit at a time): sections 1/2 = LED I / LED II steady; sections 3/4 = the
// same pair flashing (~4 Hz alternation).
static uint8_t section_led_level(uint8_t sec) {
  const bool flash = (sec >= 2) && ((millis() >> 7) & 1);
  return (uint8_t)((sec & 1) ^ (flash ? 1 : 0));
}

// Step LEDs while FUNCTION is held: implemented tools lit, the active one blinking.
static uint16_t build_map_frame() {
  const bool blink8 = (millis() >> 6) & 1;
  uint16_t f = TOOLS_IMPLEMENTED;
  if (s_tool && blink8) f &= (uint16_t)~led_bit((uint8_t)(s_tool - 1));
  return f;
}

// Step LEDs for the active tool (FUNCTION released).
static uint16_t build_tool_frame(uint8_t tool) {
  const bool blink8 = (millis() >> 6) & 1;
  switch (tool) {
    case TOOL_LENGTH: {                                         // global blinks, per-pattern solid
      uint8_t L;
      bool blinkMark;
      bool slowMark = false;                                    // slow pulse = polymeter row
      if (tapB.held()) {                                        // TAP view: selected voice's length
        const uint8_t ri = instDb.value & 7;
        const uint8_t il = eng.cur().ilen[ri];
        L = il ? il : eng.cur().length;
        blinkMark = (il == 0);                                  // fast blink = following the pattern
        slowMark  = il && eng.GetInstPoly(ri);                  // slow pulse = row is polymeter
      } else {
        L = eng.global_len ? eng.global_len : eng.cur().length;
        blinkMark = (eng.global_len != 0);                      // blinking = global override active
      }
      if (((L - 1) >> 4) != disp_section) return 0;             // length marker not in this section
      const uint16_t b = led_bit((uint8_t)((L - 1) & 15));
      if (slowMark)  return ((millis() >> 8) & 1) ? b : 0;      // ~2 Hz pulse
      return blinkMark ? (blink8 ? b : 0) : b;
    }
    case TOOL_SCALE: {                                          // swing amount as a bar
      uint16_t f = 0;
      const uint8_t sw = eng.cur().swing;
      for (uint8_t i = 0; i <= sw && i < NUM_STEP_BTNS; ++i) f |= led_bit(i);
      return f;
    }
    case TOOL_MUTE: {                                           // lit = active (unmuted)
      uint16_t f = 0;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
        if (!(eng.mute_mask & (1 << i))) f |= led_bit(i);
      return f;
    }
    case TOOL_DIR:
      return led_bit((uint8_t)(eng.play_dir & 3));              // one of steps 1-4
    case TOOL_ACCENT:
      return eng.cur().steps[INST_ACCENT][disp_section];        // accent track, shown section
    case TOOL_PROB: {
      const uint8_t  pi = instDb.value & 7;
      const Pattern &p  = eng.cur();
      if (s_prob_sel == 0xFF) {          // pick: voice's steps solid, prob'd blink
        uint16_t f  = p.steps[pi][disp_section];
        uint16_t pm = 0;
        for (uint8_t k = 0; k < PROB_SLOTS; ++k)
          if (p.pstep[k] != 0xFF && (p.pval[k] >> 4) == pi && (p.pstep[k] >> 4) == disp_section)
            pm |= led_bit((uint8_t)(p.pstep[k] & 15));
        return blink8 ? (uint16_t)(f | pm) : (uint16_t)(f & ~pm);
      }
      // set: bar = the selected step's chance in 16ths (full bar = always),
      // with the selected step's LED flashing on top of it
      const uint8_t v = p.prob_get(pi, s_prob_sel);
      const uint8_t n = v ? v : NUM_STEP_BTNS;
      uint16_t f = 0;
      for (uint8_t b = 0; b < n; ++b) f |= led_bit(b);
      if (blink8) f ^= led_bit((uint8_t)(s_prob_sel & 15));
      return f;
    }
    case TOOL_COPY:                                             // 1 clear / 2 copy / 3 paste
      return led_bit(0) | led_bit(1) | led_bit(2);
    case TOOL_XFORM:                                            // 1 rotL / 2 rotR / 3 rev / 4 rand
      return led_bit(0) | led_bit(1) | led_bit(2) | led_bit(3);
    case TOOL_RATCHET: {
      const uint8_t  pi = instDb.value & 7;
      const Pattern &p  = eng.cur();
      if (s_ratchet_sel == 0xFF) {       // pick: voice's steps solid, ratcheted blink
        uint16_t f  = p.steps[pi][disp_section];
        uint16_t rm = 0;
        for (uint8_t k = 0; k < RATCHET_SLOTS; ++k)
          if (p.rstep[k] != 0xFF && (p.rval[k] >> 4) == pi && (p.rstep[k] >> 4) == disp_section)
            rm |= led_bit((uint8_t)(p.rstep[k] & 15));
        return blink8 ? (uint16_t)(f | rm) : (uint16_t)(f & ~rm);
      }
      // set: bar = the selected step's extra-hit count (1=2x, 2=3x, 3=4x; none =
      // single hit), with the selected step's LED flashing on top of it
      const uint8_t v = p.ratchet_get(pi, s_ratchet_sel);
      uint16_t f = 0;
      for (uint8_t b = 0; b < v; ++b) f |= led_bit(b);
      if (blink8) f ^= led_bit((uint8_t)(s_ratchet_sel & 15));
      return f;
    }
    case TOOL_ARP: {                                            // bar = number of arp notes
      uint16_t f = 0;
      for (uint8_t i = 0; i < eng.arp_len && i < NUM_STEP_BTNS; ++i) f |= led_bit(i);
      return f;
    }
    case TOOL_RESLICE:                                          // the stuttering playhead (in section)
      return (eng.running && eng.step >= 0 && ((uint8_t)eng.step >> 4) == disp_section)
                 ? led_bit((uint8_t)(eng.step & 15)) : 0;
    case TOOL_POLY: {                                           // steps 1-8 lit = row is polymeter
      uint16_t f = 0;
      for (uint8_t i = 0; i < NUM_INSTRUMENTS; ++i)
        if (eng.GetInstPoly(i)) f |= led_bit(i);
      return f;
    }
    case TOOL_GEN: {                                            // show the voice's steps so a
      const uint8_t gi = instDb.value & 7;                      // TAP-generated pattern is visible
      uint16_t f = eng.cur().steps[gi][disp_section];
      const uint8_t el = inst_disp_len(gi);
      const int16_t vis = (int16_t)el - (int16_t)(disp_section * NUM_STEP_BTNS);
      if (vis <= 0)                 f  = 0;
      else if (vis < NUM_STEP_BTNS) f &= (uint16_t)((1u << vis) - 1);
      const uint8_t ph = eng.inst_playhead(gi);
      if (ph != 0xFF && (ph >> 4) == disp_section) f ^= led_bit((uint8_t)(ph & 15));
      return f;
    }
    default:
      return blink8 ? led_bit((uint8_t)(tool - 1)) : 0;        // reserved / TAP-driven: blink slot
  }
}

// Step LEDs while the config menu is open: lit shows current settings.
static uint16_t build_menu_frame() {
  uint16_t f = led_bit(9);                                       // step 10 = save
  if (g_settings.out_mode == OUT_MODE_THRU)        f |= led_bit(2);   // step 3 = THRU
  if (g_settings.clock_source == CLK_SRC_INTERNAL) f |= led_bit(7);   // step 8 = INTERNAL clock
#ifdef SUPEROS_COMBINED
  f |= led_bit(GSHARP_STEP);          // step 9 solid = SuperOS is the running firmware
#endif
  return f;
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
  for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s) stepB[s].push(panel.step(s));
  clearB.push(panel.clear());
  fnB.push(panel.function());
  groupB.push(panel.group());
  tapB.push(panel.write_tap());
  runB.push(panel.run());
  // The ~1.5 ms COMMON-TRIG pulse (and the instrument-data lines it strobes)
  // couples into the gated PA status sense (see hw.h col_us note), so a PA3
  // sample taken while a pulse is in flight can read a phantom level. The
  // pulse spans 1-2 scan passes — enough to feed the debouncer a fake
  // low-high-high and step the sequencer on a phantom clock edge. A dense
  // pattern (e.g. the GEN tool's random fill) fires big voice stacks on
  // nearly every step, so the phantom edges walked it audibly off time.
  // Hold the clock debouncer at its last level for those samples: a real
  // edge under the pulse is then seen a pass or two late (bounded jitter,
  // corrected at the next clean edge) instead of firing an extra step.
  // Checked BEFORE eng.Service() below, so this reflects the pulse state
  // during the scan that produced the sample.
  clkB.push(eng.PulseActive() ? (clkB.state & 1) : panel.tempo_clk());
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

#ifdef SUPEROS_COMBINED
  // SysEx 0x4D: reboot into the D650C emulator. Flush edits to flash first.
  if (midi_take_fw_switch() == FW_D650) {
    save_dirty(eng);
    combined_switch_firmware(FW_D650);
  }
#endif
  // In INTERNAL/DIN clock mode the MIDI clock + transport are ignored entirely:
  // the 606 always runs from its own TEMPO knob / rear DIN-sync jack. (SysEx,
  // notes and program change in midi_rx_poll are unaffected.)
  if (g_settings.clock_source != CLK_SRC_MIDI) {
    mc.pulses = 0; mc.transport = false; mc.started = false; mc.stopped = false;
  }
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

  bool replay_tick = false;   // start landed just after a swallowed clock edge
  if ((want_run && !s_want_run) || (mc.started && want_run)) {       // start / resync
    eng.Start(mode == TRACK_PLAY || mode == TRACK_WRITE);
    { const uint8_t sreg = SREG; cli(); s_clk_running = true; SREG = sreg; }
    midiRT(0xFA);
    // A clock edge recognized within the grace window before this start was
    // swallowed by the not-yet-running engine (see CLK_EDGE_GRACE_US): replay
    // it below so step 1 fires on the master's downbeat pulse, not the next
    // one. An edge rising in THIS pass is counted normally by section 5.
    replay_tick = !ext_sync && !clkB.rising() && s_clk_edge_seen &&
                  (uint32_t)(micros() - s_clk_edge_us) < CLK_EDGE_GRACE_US;
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
    s_clk_edge_us   = micros();
    s_clk_edge_seen = true;
  } else if (replay_tick) {
    // the pre-start edge swallowed by the debounce race: tick the engine now.
    // Its 0xF8 already went out and the tempo tracker already saw it when the
    // edge itself was polled; only the engine tick was lost.
    ticks = 1;
    s_clk_edge_seen = false;
  } else if (s_clk_edge_seen &&
             (uint32_t)(micros() - s_clk_edge_us) >= CLK_EDGE_GRACE_US) {
    s_clk_edge_seen = false;   // stale: also guards the ~71 min micros() wrap
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

  // 5b. Config menu. Press FUNCTION + CLEAR together to enter (either order), press
  // either again to leave. The entry and exit are one if/else-if so the CLEAR that
  // opens it can't also close it in the same pass. s_menu_hold keeps the mode
  // handlers frozen until the exit chord is released (so a still-held CLEAR can't
  // reach a mode handler). TAP held is the bootloader combo, so skip entry then.
  if (fnB.rising()) s_fn_step_used = false;

  if (!s_menu && !s_menu_hold && !tapB.held() &&
      fnB.held() && clearB.held() && (fnB.rising() || clearB.rising())) {
    s_menu = true;
    s_fn_step_used = true;
  } else if (s_menu && (clearB.rising() || fnB.rising())) {
    s_menu = false;
    s_menu_hold = true;
  }
  if (s_menu_hold && !clearB.held() && !fnB.held()) s_menu_hold = false;

  if (s_menu || s_menu_hold) {
    if (s_menu) {
#ifdef SUPEROS_COMBINED
      if (stepB[GSHARP_STEP].rising()) {         // G#: boot the D650C emulator
        save_dirty(eng);
        combined_switch_firmware(FW_D650);       // does not return
      }
#endif
      handle_config_menu();
    }
    midi_tx_service(eng);
    if (midi_take_save_request(eng)) save_dirty(eng);
    if (midi_take_settings_save(eng)) save_settings(g_settings);
    eng.SetGroupLed(disp_group);
    frame = build_menu_frame();
    return;
  }

  // FUNCTION tools live only in PATTERN WRITE. In PLAY / TRACK modes the step
  // buttons select/chain patterns or edit tracks and FUNCTION is left alone.
  const bool tools_ok = (mode == PATTERN_WRITE);

  // FUNCTION held: step buttons pick a tool; releasing with no step pressed exits
  // the current tool back to the base mode.
  if (tools_ok && fnB.held()) {
    for (uint8_t s = 0; s < NUM_STEP_BTNS; ++s)
      if (stepB[s].rising()) {
        s_tool = (uint8_t)(s + 1);
        s_fn_step_used = true;
        s_scale_ref = scaleDb.value;   // tools react only to SCALE moves made
                                       // after entry, never to its resting spot
        if (s_tool == TOOL_LENGTH) {   // open on the section holding the length
          const uint8_t L = eng.global_len ? eng.global_len : eng.cur().length;
          disp_section = (uint8_t)(((L - 1) >> 4) & 3);
        }
      }
  }
  if (fnB.falling()) {
    s_fn_up_ms = millis();               // start the stray-step swallow window
    if (!s_fn_step_used) s_tool = TOOL_NONE;
  }

  // 6. dispatch: base mode, or the active tool (frozen while FUNCTION is held)
  if (mode != prev_mode) {
    anchor = -1;
    s_tool = TOOL_NONE;                 // turning the dial lands you in a base mode
    disp_section = 0;                   // and on section 1
    if (!eng.running) save_dirty(eng);
    prev_mode = mode;
  }
  if (tools_ok && fnB.held()) {
    // selecting a tool / viewing the map: base + tool handlers frozen
  } else if (tools_ok && s_tool != TOOL_NONE) {
    handle_tool(s_tool, inst);
  } else {
    switch (mode) {
      case PATTERN_WRITE: handle_pattern_write(inst); break;
      case PATTERN_PLAY:  handle_pattern_play();      break;
      case TRACK_PLAY:
      case TRACK_WRITE:   handle_track_modes(mode, inst); break;
    }
  }

  // The arp is momentary, not a latch: leaving the ARP tool (tap FUNCTION,
  // switch tools, or turn the mode dial) clears it so the pattern comes back.
  // A latched arp full of rests read as a dead machine on the bench.
  static uint8_t s_tool_prev = TOOL_NONE;
  if (s_tool_prev == TOOL_ARP && s_tool != TOOL_ARP) eng.ArpClear();
  if (s_tool != s_tool_prev) {                    // PROB/RATCHET re-open in pick phase
    s_prob_sel    = 0xFF;
    s_ratchet_sel = 0xFF;
  }
  s_tool_prev = s_tool;

  // 7. web-editor MIDI link, outgoing side: broadcast the selected pattern
  // (0x1E), service queued pattern/track dumps, and pump the TX queue. Incoming
  // SysEx/notes were already handled in step 3b. Pushed data persists once the
  // line goes idle while stopped (never mid-transfer — each SPM page write
  // halts the CPU and would drop incoming MIDI bytes).
  midi_tx_service(eng);
  if (midi_take_save_request(eng)) save_dirty(eng);
  if (midi_take_settings_save(eng)) save_settings(g_settings);   // editor changed a global

  // 8. PATTERN GROUP LEDs + next display frame. Whenever sections are in play
  // (pattern write running, or a section-aware tool active) the group LEDs show
  // WHICH 16-step section is being viewed/edited, per section_led_level's ladder.
  // Every other context shows the pattern bank (I/II) like the stock 606.
  const bool section_ctx = (mode == PATTERN_WRITE) &&
      (eng.running || (s_tool != TOOL_NONE && tool_uses_sections(s_tool)));
  eng.SetGroupLed(section_ctx ? section_led_level(disp_section) : disp_group);
  if (tools_ok && fnB.held())               frame = build_map_frame();
  else if (tools_ok && s_tool != TOOL_NONE) frame = build_tool_frame(s_tool);
  else                                      frame = build_frame(mode, inst);
}
