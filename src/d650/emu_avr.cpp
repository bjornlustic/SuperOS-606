// emu_avr.cpp -- AVR bridge for the TR-606 D650C-128 emulator (combined build).
//
// Runs the MAME-verified ucom4 core + 606 d650_host at the real machine rate
// (454.545 kHz LC / 4 = 113.636k machine cycles/s) on the AT90USB1286 at
// 16 MHz, wiring the emulated µPD650 ports to the real TR-606 through the
// same socket pins SuperOS uses (src/pins.h):
//
//   emulated PG/PH  -> AVR PORTF        (step-LED matrix + scan selects,
//                                        the ROM does its own multiplexing)
//   emulated PF/PD  -> AVR PORTC        (instrument data LT/SD/BD/AC + PD)
//   emulated PE     -> AVR PE6/PE7/PD0/PD1  (CH/OH/CY/HT + I/II indicator)
//   emulated PI2    -> AVR PE1          (COMMON TRIG)
//   emulated PI0/PI1 (RAM WE/CE) are NOT mirrored: the physical uPD444s stay
//   deselected (PI1 pin low) and the emulated 2048-nibble store in SRAM is
//   authoritative; it persists to the internal EEPROM (combined.h map).
//
// Keys are scanned into a snapshot every 4 ms (direct PINB reads like the
// 303 bridge; all 8 matrix lines are PINB, selects are PORTF low nibble) and
// served to the emulated PA/PB reads by d650_host. The tempo/DIN clock is
// NOT snapshotted: it is edge-detected at ~1.4 kHz and re-synthesized as
// sampler-visible pulses on the host's read_clock hook (the 303's emu_sync
// pulse stretcher — see the clock block below), so the top of the tempo-knob
// range can't out-run the ROM's ~1.8 ms status sampling. MIDI clock takes
// over automatically while 0xF8 is arriving (like SuperOS).
//
// Pacing (ported from the 303 bridge, measured there at 113.6 kcyc/s with
// ~9% margin): 16.16 fixed-point cycle budget from micros(), short batches
// (drop deep backlog instead of catching up), /INT delivered in the
// emulated-cycle domain at exactly 555.55 Hz (1.8 ms, schematic p.4).
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "pins.h"
#include "hw.h"
#include "combined.h"
extern "C" {
#include "emu/ucom4.h"
#include "emu/d650_host.h"
}
#include "rom_store.h"
#ifdef D650_ROM_EMBEDDED
#include "rom_embed.h"   // generated locally, gitignored (personal builds only)
#endif

#ifndef SUPEROS_VERSION
#define SUPEROS_VERSION "dev"     // injected by tools/version_flag.py
#endif

static d650_host H;

// The mask ROM is user-supplied (see rom_store.h): loaded from EEPROM into
// SRAM at boot and executed from there (D650_ROM_IN_RAM). If no valid ROM is
// stored, emu_loop sits in an upload-wait mode until one arrives over MIDI.
static uint8_t s_rom[D650_ROM_SIZE];
static bool    s_rom_valid = false;   // EEPROM copy present + checksummed
static bool    s_rom_busy  = false;   // transfer in progress: interpreter frozen
static RomRx   s_rrx;

// ---------------------------------------------------------------------------
// MIDI note out: COMMON TRIG rising edge -> Note On per data bit, Note Off
// ~20 ms later (the 606's own trigger-out pulse width). Accent = velocity.
// ---------------------------------------------------------------------------
// mask bit -> note: lat_f bit0..2 = LT,SD,BD (bit3 = accent), lat_e = CH,OH,CY,HT
static const uint8_t NOTE_OF_BIT[7] = { 45, 38, 36, 42, 46, 49, 50 };
static uint8_t  s_off_note[8], s_off_n = 0;   // pending note-offs (one batch)
static uint32_t s_off_ms = 0;

static inline void tx3(uint8_t a, uint8_t b, uint8_t c) {
  if (Serial1.availableForWrite() >= 3) {      // drop, never block the loop
    Serial1.write(a); Serial1.write(b); Serial1.write(c);
  }
#ifdef SUPEROS_USB_MIDI
  // mirror the drum notes over USB-C (drops instantly when not enumerated)
  if ((a & 0xF0) == 0x90)      usbMIDI.sendNoteOn(b, c, (uint8_t)((a & 0x0F) + 1));
  else if ((a & 0xF0) == 0x80) usbMIDI.sendNoteOff(b, c, (uint8_t)((a & 0x0F) + 1));
#endif
}
static void note_out_fire(uint8_t pf, uint8_t pe) {
  const uint8_t vel = (pf & 8) ? 127 : 96;
  const uint8_t mask = (uint8_t)((pf & 7) | (pe << 3));
  if (!mask) return;
  if (s_off_n) {                               // previous batch still pending
    for (uint8_t i = 0; i < s_off_n; ++i) tx3(0x80, s_off_note[i], 0);
    s_off_n = 0;
  }
  for (uint8_t b = 0; b < 7; ++b)
    if (mask & (1 << b)) {
      tx3(0x90, NOTE_OF_BIT[b], vel);
      if (s_off_n < sizeof s_off_note) s_off_note[s_off_n++] = NOTE_OF_BIT[b];
    }
  s_off_ms = millis() + 20;
}
static void note_out_service() {
  if (s_off_n && (int32_t)(millis() - s_off_ms) >= 0) {
    for (uint8_t i = 0; i < s_off_n; ++i) tx3(0x80, s_off_note[i], 0);
    s_off_n = 0;
  }
}

// ---------------------------------------------------------------------------
// Port mirroring: emulated latches -> real pins
// ---------------------------------------------------------------------------
static uint8_t s_prev_pi2 = 0;

static void hook_port(void *, int port, uint8_t v) {
  switch (port) {
    case UCOM4_PORTG:                       // step-LED rows -> PORTF high nibble
      PORTF = (uint8_t)((PORTF & 0x0F) | (v << 4));
      break;
    case UCOM4_PORTH:                       // scan selects -> PORTF low nibble
      PORTF = (uint8_t)((PORTF & 0xF0) | (v & 0x0F));
      break;
    case UCOM4_PORTF:                       // LT/SD/BD/AC data -> AVR PC4-7
      PORTC = (uint8_t)((PORTC & 0x0F) | (v << 4));
      break;
    case UCOM4_PORTD:                       // µPD PD -> AVR PC0-3
      PORTC = (uint8_t)((PORTC & 0xF0) | (v & 0x0F));
      break;
    case UCOM4_PORTE: {                     // CH/OH/CY/HT + I/II
      uint8_t e = PORTE & (uint8_t)~0xC2;   // keep PE1 (TRIG) out of this write
      if (v & 1) e |= 0x40;                 // µPD PE0 -> AVR PE6
      if (v & 2) e |= 0x80;                 // µPD PE1 -> AVR PE7
      PORTE = (uint8_t)(e | (PORTE & 0x02));
      uint8_t d = PORTD & (uint8_t)~0x03;   // µPD PE2/PE3 -> AVR PD0/PD1
      PORTD = (uint8_t)(d | ((v >> 2) & 3));
      break;
    }
    case UCOM4_PORTI: {                     // PI2 = COMMON TRIG -> AVR PE1
      const uint8_t pi2 = (uint8_t)(v & 4);
      if (pi2) PORTE |= 0x02; else PORTE &= (uint8_t)~0x02;
      if (pi2 && !s_prev_pi2) note_out_fire(H.lat_f, H.lat_e);
      s_prev_pi2 = pi2;
      break;                                // PI0/PI1 never reach real pins
    }
    default: break;                         // PORTC (RAM data) stays quiescent
  }
}

// ---------------------------------------------------------------------------
// Tempo/DIN/MIDI clock -> one pulse stretcher (the SuperOS-303 emu_sync
// scheme, which fixed "tempo knob fully right slows the tempo DOWN" there).
// Feeding the raw line level to the ROM fails at the top of the knob range:
// the oscillator's pulses get too fast/narrow for the ROM's ~1.8 ms status
// sampling, edges get eaten, and more knob = fewer seen ticks = slower
// playback. Instead the physical line is sampled at ~1.4 kHz (parking the
// selects for 15 us; the line is actively driven, so it needs none of the
// key lines' settle time), each rising edge is queued (2 deep, like the
// 303), and the queue is re-synthesized as clean pulses on the level that
// emu_read_clock() serves to the ROM's status reads: high for 2x the /INT
// sampler period, then low for at least 1x, so the ROM can never miss
// either phase. Both widths scale with the interpreter slowdown
// (16 MHz / F_CPU): at 4 MHz the ROM samples 4x slower in real time, so the
// pulses stretch 4x with it (top-of-knob tempo still saturates in the 4 MHz
// slow-motion build; that ceiling is the interpreter's, not the clock's).
// MIDI clock (0xF8 within 300 ms) takes over automatically: its edges feed
// the same stretcher and the physical line is ignored while it is live.
// Anything the 1.4 kHz sampler can't see (the too-narrow-while-stopped
// pulses), the original CPU could not see either.
// ---------------------------------------------------------------------------
void emu_pcint() { /* PCINT not used in emulator mode (see above) */ }

static const uint32_t EMU_SLOWDOWN       = 16000000UL / F_CPU;
static const uint32_t CLK_PULSE_US       = 3600UL * EMU_SLOWDOWN;
static const uint32_t CLK_MIN_LOW_US     = 1800UL * EMU_SLOWDOWN;
static const uint32_t CLK_SAMPLE_GAP_US  = 700;

static uint8_t  s_clk_level = 0;            // the level the ROM reads
static uint8_t  s_clk_pending = 0;          // queued edges (max 2)
static uint8_t  s_clk_raw_prev = 0;
static uint32_t s_clk_fall_us = 0;          // scheduled end of the high phase
static uint32_t s_clk_low_us = 0;           // earliest next rise
static uint32_t s_clk_sample_us = 0;
static uint8_t  s_midi_run = 0;
static uint32_t s_last_f8_ms = 0;           // MIDI clock liveness window

static inline bool midi_clock_live() { return (millis() - s_last_f8_ms) < 300; }
static inline void clk_queue_edge()  { if (s_clk_pending < 2) s_clk_pending++; }

// The stretched level, served at the exact instant the ROM reads its status
// group (d650_drivers::read_clock).
static uint8_t emu_read_clock(void *) { return s_clk_level; }

// Physical line sampler. Runs between interpreter batches (never concurrent
// with emulated port I/O), so parking the selects for 15 us is safe: the
// mirrored LED/scan state is restored before the ROM runs again.
static void clock_sample(uint32_t now_us) {
  if ((int32_t)(now_us - s_clk_sample_us) < (int32_t)CLK_SAMPLE_GAP_US) return;
  s_clk_sample_us = now_us;
  const uint8_t save_f = PORTF;
  PORTF = 0x0F;                             // selects parked: PB3 = clock line
  delayMicroseconds(15);
  const uint8_t raw = (uint8_t)((PINB >> 3) & 1);
  PORTF = save_f;
  if (raw && !s_clk_raw_prev && !midi_clock_live()) clk_queue_edge();
  s_clk_raw_prev = raw;
}

// Pulse stretcher: turn queued edges into sampler-visible highs + lows.
static void clock_service(uint32_t now_us) {
  if (s_clk_level && (int32_t)(now_us - s_clk_fall_us) >= 0) {
    s_clk_level  = 0;
    s_clk_low_us = now_us + CLK_MIN_LOW_US;
  }
  if (s_clk_pending && !s_clk_level && (int32_t)(now_us - s_clk_low_us) >= 0) {
    s_clk_pending--;
    s_clk_level   = 1;
    s_clk_fall_us = now_us + CLK_PULSE_US;
  }
}
static void midi_clock_edge(uint32_t now_us) {    // one 0xF8 = one 24-PPQN edge
  (void)now_us;
  s_last_f8_ms = millis();
  clk_queue_edge();
}

// ---------------------------------------------------------------------------
// MIDI in: real-time transport/clock + the firmware-switch SysEx
// ---------------------------------------------------------------------------
static uint8_t s_sx[4]; static uint8_t s_sx_n = 0; static uint8_t s_sx_on = 0;
static void patt_flush_blocking();          // defined with the sweeper below

#ifdef SUPEROS_USB_MIDI
// USB SysEx: scan the chunked stream for the firmware switch (F0 7D 4D [fw]
// F7 with fw = 0/absent -> SuperOS). Chunks may split anywhere; this state
// machine spans them. Everything else over USB SysEx is ignored, exactly like
// the DIN path: the original firmware only speaks the switch command.
static uint8_t s_usx_state = 0;   // 0 idle, 1 F0, 2 7D, 3 4D(+fw), 0xFF other cmd
static uint8_t s_usx_fw = 0;
static void usb_sysex_chunk(const uint8_t *d, uint16_t n, bool complete) {
  (void)complete;
  for (uint16_t i = 0; i < n; ++i) {
    const uint8_t b = d[i];
    if (b == 0xF0) { s_usx_state = 1; s_usx_fw = FW_SUPEROS; continue; }
    if (b == 0xF7) {
      if (s_usx_state == 3 && s_usx_fw == FW_SUPEROS) {
        patt_flush_blocking();
        combined_switch_firmware(FW_SUPEROS);   // does not return
      }
      s_usx_state = 0;
      continue;
    }
    if      (s_usx_state == 1 && b == 0x7D) s_usx_state = 2;
    else if (s_usx_state == 2 && b == 0x4D) s_usx_state = 3;
    else if (s_usx_state == 3)              s_usx_fw = b;
    else if (s_usx_state)                   s_usx_state = 0xFF;
  }
}
#endif

static void midi_poll(uint32_t now_us) {
#ifdef SUPEROS_USB_MIDI
  // USB-C parity with DIN: clock + transport drive the emulated CLOCK/RUN
  // lines, SysEx feeds the firmware-switch matcher (via the chunk handler).
  while (usbMIDI.read()) {
    const uint8_t t = usbMIDI.getType();
    if (t == 0xF8) midi_clock_edge(now_us);
    else if (t == 0xFA || t == 0xFB) s_midi_run = 1;
    else if (t == 0xFC) s_midi_run = 0;
  }
#endif
  while (Serial1.available()) {
    const uint8_t b = (uint8_t)Serial1.read();
    if (b >= 0xF8) {                        // real-time: legal anywhere
      if (b == 0xF8) midi_clock_edge(now_us);
      else if (b == 0xFA || b == 0xFB) s_midi_run = 1;
      else if (b == 0xFC) s_midi_run = 0;
      continue;
    }
    // Mask-ROM upload (rom_store.h format). The receiver decodes straight
    // into s_rom, so a confirmed block start freezes the interpreter; a
    // failed or aborted transfer restores s_rom from the EEPROM copy.
    switch (s_rrx.feed(b, millis())) {
      case ROMRX_STARTED: s_rom_busy = true; break;
      case ROMRX_BLOCK:   break;            // stay frozen for the next block
      case ROMRX_DONE:                       // persist + clean reboot into emu
        patt_flush_blocking();
        rom_save(s_rom);
        combined_switch_firmware(FW_D650);   // does not return
        break;
      case ROMRX_ERROR:
        if (s_rom_valid && s_rom_busy) rom_load(s_rom);
        s_rom_busy = false;
        break;
      default: break;
    }
    if (b == 0xF0) { s_sx_on = 1; s_sx_n = 0; continue; }
    if (b == 0xF7) {
      // F0 7D 4D <fw> F7 -> switch firmware (combined.h)
      if (s_sx_on && s_sx_n >= 2 && s_sx[0] == 0x7D && s_sx[1] == 0x4D) {
        const uint8_t fw = (s_sx_n >= 3) ? s_sx[2] : FW_SUPEROS;
        if (fw == FW_SUPEROS) {
          patt_flush_blocking();            // the store must land before reboot
          combined_switch_firmware(FW_SUPEROS);
        }
      }
      // F0 7D 34 F7 -> version request (same command SuperOS answers; the
      // suffix tells the editor the emulator side is running). Best-effort:
      // skipped if the TX FIFO lacks room, never blocks the interpreter.
      if (s_sx_on && s_sx_n >= 2 && s_sx[0] == 0x7D && s_sx[1] == 0x34) {
        static const char v[] = SUPEROS_VERSION " D650C";
        if (Serial1.availableForWrite() >= (int)sizeof(v) + 3) {
          Serial1.write((uint8_t)0xF0); Serial1.write((uint8_t)0x7D); Serial1.write((uint8_t)0x35);
          for (uint8_t i = 0; v[i]; ++i) Serial1.write((uint8_t)(v[i] & 0x7F));
          Serial1.write((uint8_t)0xF7);
        }
      }
      // F0 7D 36 F7 -> D650C ROM status (0x37 reply, same codes as SuperOS:
      // 0 none, 1 EEPROM upload, 2 embedded build). s_rom_valid is what the
      // interpreter actually runs; the magic byte says where it came from.
      if (s_sx_on && s_sx_n >= 2 && s_sx[0] == 0x7D && s_sx[1] == 0x36) {
        uint8_t st = 0;
        if (s_rom_valid)
          st = (eeprom_read_byte(EE_ROM_MAGIC) == EE_ROM_MAGIC_VAL) ? 1 : 2;
        if (Serial1.availableForWrite() >= 5) {
          Serial1.write((uint8_t)0xF0); Serial1.write((uint8_t)0x7D);
          Serial1.write((uint8_t)0x37); Serial1.write(st);
          Serial1.write((uint8_t)0xF7);
        }
      }
      s_sx_on = 0; s_sx_n = 0;
      continue;
    }
    if (b & 0x80) { s_sx_on = 0; s_sx_n = 0; continue; }   // other status: ignore
    if (s_sx_on && s_sx_n < sizeof s_sx) s_sx[s_sx_n++] = b;
  }
}

// ---------------------------------------------------------------------------
// Persistence: internal-EEPROM non-blocking byte sweeper (303 combined scheme;
// a byte program runs ~3.3 ms in hardware while the interpreter keeps going)
// ---------------------------------------------------------------------------
static bool     s_pending = false;
static uint16_t s_ee_cursor = 0;
static uint32_t s_quiet_ms = 0, s_ui_ms = 0;

static bool patt_save_step() {
  if (!eeprom_is_ready()) return false;
  uint8_t scanned = 0;
  while (s_ee_cursor < D650_EXT_BYTES && scanned < 64) {
    const uint8_t want = H.ext[s_ee_cursor];
    if (eeprom_read_byte(EE_EMU_PATT + s_ee_cursor) != want) {
      eeprom_write_byte(EE_EMU_PATT + s_ee_cursor, want);
      s_ee_cursor++;
      return true;
    }
    s_ee_cursor++; scanned++;
  }
  if (s_ee_cursor >= D650_EXT_BYTES) { s_ee_cursor = 0; s_pending = false; }
  return false;
}
static void patt_flush_blocking() {         // worst case ~3.4 s (full store)
  if (d650_dirty(&H)) { d650_clear_dirty(&H); s_pending = true; s_ee_cursor = 0; }
  while (s_pending) { wdt_reset(); patt_save_step(); }
}

// Seed the whole uPD444 store to the empty state (all-zero nibbles). Blocking,
// ~3.4 s worst case on a flash-erased store, but eeprom_update_byte skips bytes
// that already match, so a clean store costs only reads. Runs once, the first
// boot after the store is (re)initialized. H.ext is already zero from d650_init.
static void patt_seed_blocking() {
  for (uint16_t i = 0; i < D650_EXT_BYTES; ++i) {
    eeprom_update_byte(EE_EMU_PATT + i, 0);
    if ((i & 0x3F) == 0) wdt_reset();
  }
}

static void patt_load() {
  if (eeprom_read_byte(EE_EMU_MAGIC) != EE_EMU_MAGIC_VAL) {
    // Fresh or migrated store: write the empty state NOW, magic LAST, so a power
    // loss mid-seed leaves the magic virgin and the seed simply repeats. The old
    // code deferred this via s_pending, but upload-wait mode never runs the
    // saver, so the store could stay flash-erased 0xFF (= every step of every
    // pattern filled) while the magic already claimed "initialized" — the
    // ROM-upload "all patterns filled" bug. Seeding synchronously fixes it.
    patt_seed_blocking();
    eeprom_update_byte(EE_EMU_MAGIC, EE_EMU_MAGIC_VAL);
    return;                                 // H.ext already zeroed by d650_init
  }
  eeprom_read_block(H.ext, EE_EMU_PATT, D650_EXT_BYTES);
}

// ---------------------------------------------------------------------------
// Input scan: direct-port matrix poll into the snapshot. Read timing matches
// the schedule proven on this hardware by the working 4 MHz SuperOS build
// (see the note in src/hw.h): PB keys valid by +15 us after the select, but
// the PA lines decay slowly through the 15 K pulldowns and only read clean at
// ~+50..90 us — the 303's +29 us was NOT enough on the 606's board (phantom
// RUN/CLEAR/mode levels). Status group read at +40 us after selects restore.
// The slower scan runs every 4 ms (~11% CPU) instead of 2 ms; key latency
// stays well under a scan frame of the emulated ROM.
// ---------------------------------------------------------------------------
static uint8_t s_db[D650_IN_COUNT];         // 3-sample debounce shift registers

static inline void db_push(uint8_t i, uint8_t bit) {
  s_db[i] = (uint8_t)(((s_db[i] << 1) | bit) & 7);
}
static void poll_matrix() {
  const uint8_t save_f = PORTF;             // emulated PG/PH state
  PORTF = 0x0F;
  delayMicroseconds(40);
  uint8_t v = PINB;
  for (uint8_t j = 0; j < 4; ++j) db_push(32 + j, (v >> j) & 1);
  for (uint8_t s = 0; s < 4; ++s) {
    PORTF = (uint8_t)(0x0F & ~(1 << s));
    delayMicroseconds(15);
    const uint8_t vb = PINB;                // PB nibble: step keys
    delayMicroseconds(70);                  // PA residual decayed (+85 total)
    const uint8_t va = PINB;                // PA nibble: rotaries/buttons
    for (uint8_t j = 0; j < 4; ++j) {
      db_push((uint8_t)(s * 4 + j), (vb >> (4 + j)) & 1);
      db_push((uint8_t)(16 + s * 4 + j), (va >> j) & 1);
    }
  }
  PORTF = save_f;
}
// ---------------------------------------------------------------------------
// Config menu (mirrors the SuperOS-303 combined build and SuperOS-606's own):
// FUNCTION held + CLEAR enters, CLEAR or FUNCTION exits. Inside, step 9 (the
// "G#" key) reboots into SuperOS. The ROM is frozen while the menu is up —
// its inputs are zeroed and no cycles run — so the held FUNCTION/CLEAR chord
// can't trigger original-firmware actions (last-step set, D.C. reset...).
// The step-9 LED blinks = the D650C is the running firmware (SuperOS's menu
// shows it solid, same convention as the 303).
// ---------------------------------------------------------------------------
static bool    s_menu = false;
static bool    s_menu_hold = false;         // exit chord still down: mask it
static uint8_t s_menu_prev = 0;             // bit0 FN, bit1 CLEAR, bit2 step9

static void menu_service() {
  const uint8_t fn  = s_db[29] ? 1 : 0;     // FUNCTION
  const uint8_t cl  = s_db[28] ? 1 : 0;     // CLEAR
  const uint8_t st9 = s_db[8]  ? 1 : 0;     // step 9 = "G#"
  const uint8_t tap = s_db[33] ? 1 : 0;
  const uint8_t fn_r  = fn  && !(s_menu_prev & 1);
  const uint8_t cl_r  = cl  && !(s_menu_prev & 2);
  const uint8_t st9_r = st9 && !(s_menu_prev & 4);
  s_menu_prev = (uint8_t)(fn | (cl << 1) | (st9 << 2));

  if (!s_menu) {
    if (fn && cl_r && !tap) {
      s_menu = true;
      for (uint8_t i = 0; i < D650_IN_COUNT; ++i) H.in[i] = 0;
    }
    return;
  }
  if (cl_r || fn_r) {                       // exit: restore the ROM's LED state
    s_menu = false;
    s_menu_hold = true;                     // mask the chord until released
    PORTF = (uint8_t)((H.lat_g << 4) | (H.lat_h & 0x0F));
    return;
  }
  if (st9_r) {                              // G#: boot SuperOS
    patt_flush_blocking();
    combined_switch_firmware(FW_SUPEROS);   // does not return
  }
  // menu display: blinking step-9 LED (row 2 select low, PG0 high)
  PORTF = ((millis() >> 8) & 1) ? (uint8_t)((0x0F & ~(1 << 2)) | 0x10) : 0x0F;
}

static void feed_inputs() {
  uint8_t edge = 0;
  for (uint8_t i = 0; i < D650_IN_COUNT; ++i) {
    if (i == D650_IN_CLOCK) continue;   // clock never comes from the snapshot:
                                        // emu_read_clock serves the stretcher
    const uint8_t held = s_db[i] ? 1 : 0;   // any of the last 3 samples
    if (i < 32 && held != H.in[i]) edge = 1;
    H.in[i] = held;
  }
  if (edge) s_ui_ms = millis();
  if (s_midi_run) H.in[D650_IN_RUN] = 1;    // MIDI transport ORs the panel FF
  // A CLEAR/FUNCTION press that exited the menu must never reach the ROM
  // (CLEAR clears patterns, FUNCTION shifts): mask both until released. All
  // other inputs (RUN especially) flow normally so playback resumes at once.
  if (s_menu_hold) {
    H.in[28] = H.in[29] = 0;
    if (!s_db[28] && !s_db[29]) s_menu_hold = false;
  }
}

// ---------------------------------------------------------------------------
// Mask-ROM transfer housekeeping
// ---------------------------------------------------------------------------
// Stalled transfer (sender unplugged mid-message): reset the receiver and, if
// the running ROM's buffer was already dirtied, restore it from EEPROM.
static void romrx_service() {
  if ((s_rom_busy || s_rrx.busy()) && (millis() - s_rrx.last_ms) > 2000) {
    s_rrx.reset();
    if (s_rom_valid && s_rom_busy) rom_load(s_rom);
    s_rom_busy = false;
  }
}

// Upload-wait display (no valid ROM stored): steps 1 and 9 alternate, slow
// while idle, fast while a transfer is running. Static single-column PORTF
// writes only, so nothing here can disturb the MIDI byte pump.
static void romwait_display() {
  const uint16_t period = s_rrx.busy() ? 100 : 400;
  const bool phase = (millis() / period) & 1;
  hw::LightCell(phase ? 2 : 0, 0);          // step 9 / step 1
}

// ---------------------------------------------------------------------------
// Real-time pacing (constants measured on the 303: 113636 cyc/s sustained)
// ---------------------------------------------------------------------------
static const uint32_t CYC_PER_US_Q16   = 7447;    // 113.636 kHz in 16.16
static const uint32_t INT_PERIOD_CYC_Q8 = 52364;  // 1.8 ms in machine cycles, Q8
static const uint32_t EMU_MAX_BATCH    = 384;
static const uint32_t EMU_MAX_BACKLOG  = 1024;
// Machine cycles in one clock-sample period (~700 us). Bounding each inner
// d650_run to this many cycles keeps clock_sample() on its fixed ~1.4 kHz beat
// even when a whole batch is owed (see the batch loop for why).
static const uint32_t CLK_SAMPLE_CYC   = (CLK_SAMPLE_GAP_US * CYC_PER_US_Q16) >> 16;
static uint32_t g_last_us = 0, g_budget_q16 = 0;
static int32_t  s_int_due_q8 = 0;

void emu_setup() {
#ifndef SUPEROS_USB_MIDI
  // No USB in this build: tear down the engine the Teensy core's usb_init()
  // brought up before setup(). Leaving it alive at 16 MHz wedges the unit on a
  // cold boot into emu mode (same failure mode that auto-ran SuperOS). This
  // mirrors SuperOS setup()'s usb_shutdown_hw(); the firmware-SWITCH path in
  // combined.cpp already shuts USB down, but a direct power-on into D650C does
  // not pass through it.
  UDIEN  = 0;
  UDCON  = 1;                 // detach
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
#endif
#ifdef SUPEROS_USB_MIDI
  usbMIDI.setHandleSystemExclusive(usb_sysex_chunk);   // 3-arg partial overload
#endif
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);       // same rationale as SuperOS setup()
  Serial1.begin(31250);
  hw::Init();                               // selects high, data/trig lines low
  s_rrx.buf = s_rom;
  s_rom_valid = rom_load(s_rom);            // user mask ROM: EEPROM -> SRAM
#ifdef D650_ROM_EMBEDDED
  // Personal builds (env:combined-rom) carry the user's own ROM in program
  // flash: when the EEPROM holds no valid upload, run from the embedded copy.
  // An EEPROM upload still wins, and the MIDI upload path stays available.
  if (!s_rom_valid) {
    memcpy_P(s_rom, D650_ROM_EMBED, D650_ROM_SIZE);
    s_rom_valid = true;
  }
#endif
  if (!s_rom_valid) memset(s_rom, 0, sizeof s_rom);
  d650_drivers drv = { hook_port, nullptr, emu_read_clock };
  d650_init(&H, s_rom, &drv);
  patt_load();
  g_last_us = micros();
}

void emu_loop() {
  const uint32_t now = micros();

  midi_poll(now);
  romrx_service();

  // No mask ROM stored: upload-wait mode. midi_poll above keeps consuming the
  // transfer (and still honors the F0 7D 4D 00 F7 switch back to SuperOS);
  // the interpreter has no ROM to execute. The panel stays alive enough for
  // the standard config menu: FUNCTION held + CLEAR opens it (step 9 blinks),
  // step 9 boots SuperOS, CLEAR or FUNCTION drops back to upload-wait.
  if (!s_rom_valid) {
    static uint32_t rw_poll_us = 0;
    if ((int32_t)(now - rw_poll_us) >= 0) {
      rw_poll_us = now + 4000;              // same cadence as the live scan
      poll_matrix();
      menu_service();                       // step 9 inside = boot SuperOS
    }
    if (!s_menu) romwait_display();         // menu_service draws while open
    return;
  }

  // Live transfer into the running emulator: ROM frozen (like the menu) until
  // the transfer completes (reboot) or fails (s_rom restored, flag cleared).
  if (s_rom_busy) {
    g_last_us = now;
    g_budget_q16 = 0;
    return;
  }

  clock_sample(now);
  clock_service(now);

  static uint32_t next_poll_us = 0;
  if ((int32_t)(now - next_poll_us) >= 0) {
    next_poll_us = now + 4000;              // see poll_matrix: slower, correct scan
    poll_matrix();
    menu_service();
    if (!s_menu) feed_inputs();
  }

  if (s_menu) {                             // ROM frozen while the menu is up
    g_last_us = now;                        // no cycle backlog builds
    g_budget_q16 = 0;
    note_out_service();
    if (s_pending) patt_save_step();        // menu time = ideal save time
    return;
  }

  // cycle budget: short batches, drop deep backlog (see the 303 bridge notes:
  // the ROM sequences off CLOCK edges, so dropped cycles are musically free)
  uint32_t dt = now - g_last_us; g_last_us = now;
  if (dt > 50000) dt = 50000;
  g_budget_q16 += dt * CYC_PER_US_Q16;
  uint32_t owed = g_budget_q16 >> 16;
  if (owed > EMU_MAX_BACKLOG) {
    g_budget_q16 -= (owed - EMU_MAX_BACKLOG) << 16;
    owed = EMU_MAX_BACKLOG;
  }
  if (owed > EMU_MAX_BATCH) owed = EMU_MAX_BATCH;
  g_budget_q16 -= owed << 16;

  // batched execution, chunked so each /INT edge lands on its emulated cycle,
  // and so the physical tempo-clock line is re-sampled at its fixed ~1.4 kHz
  // cadence WITHIN the batch. A full owed batch is up to EMU_MAX_BATCH cycles =
  // ~3.4 ms of a single d650_run; sampling the clock only once per emu_loop
  // (the old placement) therefore sampled the knob oscillator at ~300 Hz,
  // irregularly, which aliased near the top of the knob's range -- the "slow,
  // fast, then slow again" tempo. Capping each inner run to CLK_SAMPLE_CYC (one
  // sampler period) lets clock_sample keep its 700 us beat regardless of how
  // many cycles are owed. This is the SuperOS-303 emulator's clock fix (sample
  // the tempo line at a fixed rate decoupled from the interpreter) adapted to
  // the 606 emulator, which has no LED-refresh ISR to snapshot the line for us.
  uint32_t spent = 0;
  while (spent < owed) {
    if (s_int_due_q8 <= 0) {
      d650_clock(&H, 1); d650_clock(&H, 0);
      s_int_due_q8 += (int32_t)INT_PERIOD_CYC_Q8;
    }
    uint32_t chunk = owed - spent;
    const uint32_t due = (uint32_t)(s_int_due_q8 + 255) >> 8;
    if (chunk > due) chunk = due;
    if (chunk > CLK_SAMPLE_CYC) chunk = CLK_SAMPLE_CYC;
    const uint32_t ran = d650_run(&H, chunk);
    spent += ran;
    s_int_due_q8 -= (int32_t)(ran << 8);
    const uint32_t t = micros();
    clock_sample(t);                        // fixed ~1.4 kHz tempo-line sampling
    clock_service(t);                       // stretcher stays current mid-batch
  }
  if (spent > owed) {                       // overshoot <= one instruction
    const uint32_t oq = (spent - owed) << 16;
    g_budget_q16 = (g_budget_q16 > oq) ? (g_budget_q16 - oq) : 0;
  }

  note_out_service();

  // deferred persistence: quiet-gated, non-blocking
  const uint32_t ms_now = millis();
  if (d650_dirty(&H)) {
    d650_clear_dirty(&H);
    s_pending = true;
    s_ee_cursor = 0;
    s_quiet_ms = ms_now;
  }
  if (s_pending
      && (uint32_t)(ms_now - s_quiet_ms) >= 1500
      && (uint32_t)(ms_now - s_ui_ms) >= 1000)
    patt_save_step();
}
#endif // SUPEROS_COMBINED
