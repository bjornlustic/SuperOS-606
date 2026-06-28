// SuperOS-606 — MIDI IN/OUT implementation (see midi_api.h for the protocol)
//
// Everything here is non-blocking: the RX parser eats whatever bytes have
// arrived this loop pass, and the TX pump writes only as many bytes as the
// UART buffer has room for. A full track dump (82 bytes) therefore spreads
// across a few loop passes instead of stalling the sequencer.
#include <Arduino.h>
#include "pattern.h"
#include "engine.h"
#include "midi_api.h"
#include "settings.h"

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
static constexpr uint8_t SYX_MFR           = 0x7D;
static constexpr uint8_t CMD_REQ_PATTERN   = 0x10;
static constexpr uint8_t CMD_PATTERN_DUMP  = 0x11;
static constexpr uint8_t CMD_PUSH_PATTERN  = 0x12;
static constexpr uint8_t CMD_ACK           = 0x14;
static constexpr uint8_t CMD_STEP_POS      = 0x15;
static constexpr uint8_t CMD_STEP_UPDATE   = 0x16;
static constexpr uint8_t CMD_LENGTH_UPDATE = 0x18;
static constexpr uint8_t CMD_SCALE_UPDATE  = 0x19;
static constexpr uint8_t CMD_SELECT_CHAIN  = 0x1A;
static constexpr uint8_t CMD_SELECT_PAT    = 0x1D;
static constexpr uint8_t CMD_ACTIVE_PAT    = 0x1E;
static constexpr uint8_t CMD_REQ_TRACK     = 0x24;
static constexpr uint8_t CMD_TRACK_DUMP    = 0x25;
static constexpr uint8_t CMD_PUSH_TRACK    = 0x26;
static constexpr uint8_t CMD_REQ_SETTINGS  = 0x30;
static constexpr uint8_t CMD_SETTINGS_DUMP = 0x31;
static constexpr uint8_t CMD_SET_SETTINGS  = 0x32;

static constexpr uint8_t ACK_OK           = 0;
static constexpr uint8_t ACK_BAD_CHECKSUM = 1;
static constexpr uint8_t ACK_BAD_INDEX    = 2;
static constexpr uint8_t ACK_BAD_LENGTH   = 3;

// pack7 grows the payload by one MSB byte per 7 raw bytes
static constexpr uint8_t PACKED_PATTERN = PATTERN_BYTES + (PATTERN_BYTES + 6) / 7;  // 21
static constexpr uint8_t PACKED_TRACK   = TRACK_BYTES   + (TRACK_BYTES + 6) / 7;    // 75

// ---------------------------------------------------------------------------
// 7-bit pack / unpack + checksum (bootloader scheme)
// ---------------------------------------------------------------------------
static uint8_t pack7(const uint8_t *src, uint8_t n, uint8_t *out) {
  uint8_t oi = 0;
  for (uint8_t i = 0; i < n; i += 7) {
    const uint8_t chunk = (uint8_t)((n - i < 7) ? (n - i) : 7);
    uint8_t msb = 0;
    for (uint8_t b = 0; b < chunk; ++b)
      if (src[i + b] & 0x80) msb |= (uint8_t)1 << b;
    out[oi++] = msb;
    for (uint8_t b = 0; b < chunk; ++b) out[oi++] = src[i + b] & 0x7F;
  }
  return oi;
}

static bool unpack7(const uint8_t *in, uint8_t in_len, uint8_t *out, uint8_t out_len) {
  uint8_t oi = 0, ii = 0;
  while (oi < out_len) {
    if (ii >= in_len) return false;
    const uint8_t msb = in[ii++];
    for (uint8_t b = 0; b < 7 && oi < out_len; ++b) {
      if (ii >= in_len) return false;
      out[oi++] = in[ii++] | (((msb >> b) & 1) ? 0x80 : 0x00);
    }
  }
  return true;
}

static uint8_t xor_bytes(const uint8_t *buf, uint8_t n) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < n; ++i) x ^= buf[i];
  return x;
}

// ---------------------------------------------------------------------------
// TX: one ring buffer of complete messages
// ---------------------------------------------------------------------------
// Sized for a full track dump (82 bytes) plus a handful of note messages.
static uint8_t  txq[192];
static uint8_t  txq_head = 0, txq_tail = 0;   // ring indices
static uint8_t  txq_count = 0;

bool midi_tx_msg(const uint8_t *msg, uint8_t len) {
  if ((uint16_t)txq_count + len > sizeof(txq)) return false;   // full: drop whole message
  for (uint8_t i = 0; i < len; ++i) {
    txq[txq_tail] = msg[i];
    txq_tail = (uint8_t)((txq_tail + 1) % sizeof(txq));
  }
  txq_count += len;
  return true;
}

// Leave this many bytes free in the UART TX FIFO when pumping the queue, so the
// realtime / soft-thru direct writes below always have room and never have to
// BLOCK. A blocked write stalls the main loop, and a stalled loop lets the RX
// FIFO overflow — dropping incoming MIDI-clock pulses, which slips the sequencer
// behind the DAW. This was the "goes out of time once the hats are playing" bug:
// the extra Note On/Off traffic kept the TX FIFO full, so each clock-forward
// blocked. Realtime bytes are 1 byte each and only a few land per poll, so this
// headroom is plenty.
static constexpr uint8_t TX_RT_RESERVE = 8;

// Write one MIDI-OUT byte only if the UART has room; never block the loop. Used
// for realtime + soft-thru, where a dropped byte is far cheaper than a stall.
static inline void tx_raw(uint8_t b) {
  if (Serial1.availableForWrite() > 0) Serial1.write(b);
}

static void tx_pump() {
  int room = (int)Serial1.availableForWrite() - TX_RT_RESERVE;
  while (txq_count > 0 && room-- > 0) {
    Serial1.write(txq[txq_head]);
    txq_head = (uint8_t)((txq_head + 1) % sizeof(txq));
    txq_count--;
  }
}

// ---------------------------------------------------------------------------
// Outgoing message builders
// ---------------------------------------------------------------------------
static void send_ack(uint8_t status) {
  const uint8_t msg[5] = { 0xF0, SYX_MFR, CMD_ACK, status, 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}

void midi_send_step_position(uint8_t pat) {
  const uint8_t msg[5] = { 0xF0, SYX_MFR, CMD_STEP_POS, (uint8_t)(pat & 0x1F), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));   // advisory: ok to drop when the queue is busy
}

static void send_active_pattern(uint8_t pat) {
  const uint8_t msg[5] = { 0xF0, SYX_MFR, CMD_ACTIVE_PAT, (uint8_t)(pat & 0x1F), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}

// Settings dump (0x31): the three global config values, one 7-bit byte each.
static bool settings_save_pending = false;   // a 0x32 set awaiting a flash write

void midi_send_settings() {
  const uint8_t msg[7] = { 0xF0, SYX_MFR, CMD_SETTINGS_DUMP,
                           (uint8_t)(g_settings.midi_channel & 0x7F),
                           (uint8_t)(g_settings.clock_source & 0x7F),
                           (uint8_t)(g_settings.out_mode     & 0x7F), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}

// Panel-edit broadcasts: keep the web editor's copy live while writing on the
// 606 (step entry, tap write, chase-delete, length, scale). Tiny messages,
// human-rate — fine to drop if the queue is somehow full.
void midi_send_step_update(uint8_t pat, uint8_t inst, uint8_t step, bool on) {
  const uint8_t msg[8] = { 0xF0, SYX_MFR, CMD_STEP_UPDATE, (uint8_t)(pat & 0x1F),
                           (uint8_t)(inst & 7), (uint8_t)(step & 0x0F), (uint8_t)(on ? 1 : 0), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}
void midi_send_length_update(uint8_t pat, uint8_t len) {
  const uint8_t msg[6] = { 0xF0, SYX_MFR, CMD_LENGTH_UPDATE, (uint8_t)(pat & 0x1F),
                           (uint8_t)(len & 0x1F), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}
void midi_send_scale_update(uint8_t pat, uint8_t scale) {
  const uint8_t msg[6] = { 0xF0, SYX_MFR, CMD_SCALE_UPDATE, (uint8_t)(pat & 0x1F),
                           (uint8_t)(scale & 3), 0xF7 };
  midi_tx_msg(msg, sizeof(msg));
}

// Build + queue a pattern or track dump: F0 7D <cmd> <idx> <xor_lo> <xor_hi> <packed> F7
static bool send_dump(uint8_t cmd, uint8_t idx, const uint8_t *raw, uint8_t raw_len) {
  uint8_t msg[7 + PACKED_TRACK];   // largest case: track dump, 82 bytes total
  const uint8_t x = xor_bytes(raw, raw_len);
  msg[0] = 0xF0; msg[1] = SYX_MFR; msg[2] = cmd; msg[3] = idx;
  msg[4] = x & 0x7F; msg[5] = (uint8_t)((x >> 7) & 1);
  const uint8_t packed = pack7(raw, raw_len, msg + 6);
  msg[6 + packed] = 0xF7;
  return midi_tx_msg(msg, (uint8_t)(7 + packed));
}

// Dump requests wait here until the TX queue has room (a request burst from
// the editor must not silently lose replies).
static uint32_t pending_pat_dumps = 0;   // bit per pattern
static uint8_t  pending_trk_dumps = 0;   // bit per track

// Queue a full pattern dump (0x11) toward the editor — used after PATTERN
// CLEAR on the panel, where per-step broadcasts would take 16 messages.
void midi_send_pattern_dump(uint8_t pat) {
  if (pat < NUM_PATTERNS) pending_pat_dumps |= (uint32_t)1 << pat;
}

static void service_pending_dumps(Engine &eng) {
  uint8_t raw[TRACK_BYTES];
  for (uint8_t p = 0; p < NUM_PATTERNS && pending_pat_dumps; ++p) {
    if (!(pending_pat_dumps & ((uint32_t)1 << p))) continue;
    serialize_pattern(eng.pattern[p], raw);
    if (!send_dump(CMD_PATTERN_DUMP, p, raw, PATTERN_BYTES)) return;   // queue full, retry next pass
    pending_pat_dumps &= ~((uint32_t)1 << p);
  }
  for (uint8_t t = 0; t < NUM_TRACKS && pending_trk_dumps; ++t) {
    if (!(pending_trk_dumps & ((uint8_t)1 << t))) continue;
    serialize_track(eng.track[t], raw);
    if (!send_dump(CMD_TRACK_DUMP, t, raw, TRACK_BYTES)) return;
    pending_trk_dumps &= ~((uint8_t)1 << t);
  }
}

// ---------------------------------------------------------------------------
// RX: SysEx capture + dispatch
// ---------------------------------------------------------------------------
// Largest inner message: push track = 7D + cmd + idx + 2 xor + 75 packed = 80.
static uint8_t rx_buf[84];
static uint8_t rx_len      = 0;
static bool    rx_active   = false;   // between F0 and F7
static bool    rx_overflow = false;
static bool     save_pending = false;  // pushed data awaiting a flash save
static uint32_t last_rx_ms   = 0;      // for the idle-save gate

static void handle_push_pattern(Engine &eng) {
  // rx_buf: 7D 12 <pat> <xor_lo> <xor_hi> <packed...>
  if (rx_len < 5 + PACKED_PATTERN) { send_ack(ACK_BAD_LENGTH); return; }
  const uint8_t pat = rx_buf[2];
  if (pat >= NUM_PATTERNS) { send_ack(ACK_BAD_INDEX); return; }
  uint8_t raw[PATTERN_BYTES];
  if (!unpack7(rx_buf + 5, (uint8_t)(rx_len - 5), raw, PATTERN_BYTES)) { send_ack(ACK_BAD_LENGTH); return; }
  const uint8_t x = (uint8_t)(rx_buf[3] | (rx_buf[4] << 7));
  if (xor_bytes(raw, PATTERN_BYTES) != x) { send_ack(ACK_BAD_CHECKSUM); return; }
  deserialize_pattern(eng.pattern[pat], raw);   // clamps length/scale
  eng.mark_pat_dirty(pat);
  save_pending = true;
  send_ack(ACK_OK);
}

static void handle_push_track(Engine &eng) {
  // rx_buf: 7D 26 <trk> <xor_lo> <xor_hi> <packed...>
  if (rx_len < 5 + PACKED_TRACK) { send_ack(ACK_BAD_LENGTH); return; }
  const uint8_t trk = rx_buf[2];
  if (trk >= NUM_TRACKS) { send_ack(ACK_BAD_INDEX); return; }
  uint8_t raw[TRACK_BYTES];
  if (!unpack7(rx_buf + 5, (uint8_t)(rx_len - 5), raw, TRACK_BYTES)) { send_ack(ACK_BAD_LENGTH); return; }
  const uint8_t x = (uint8_t)(rx_buf[3] | (rx_buf[4] << 7));
  if (xor_bytes(raw, TRACK_BYTES) != x) { send_ack(ACK_BAD_CHECKSUM); return; }
  deserialize_track(eng.track[trk], raw);       // clamps len + pattern indices
  eng.mark_trk_dirty(trk);
  save_pending = true;
  send_ack(ACK_OK);
}

// Select a pattern or range chain remotely (web editor pill click / drag).
// Same semantics as the panel: immediate when stopped, queued at the next
// pattern wrap when running. The displayed group follows so the panel LEDs
// show what was just selected.
static void handle_select(Engine &eng, uint8_t &disp_group) {
  if (rx_buf[1] == CMD_SELECT_PAT) {
    if (rx_len < 3 || rx_buf[2] >= NUM_PATTERNS) return;
    eng.SelectPattern(rx_buf[2]);
    disp_group = rx_buf[2] / 16;
    return;
  }
  // CMD_SELECT_CHAIN: 7D 1A <n> <p0..pn-1>
  if (rx_len < 3) return;
  const uint8_t n = rx_buf[2];
  if (n < 1 || n > CHAIN_MAX || rx_len < 3 + n) return;
  uint8_t pats[CHAIN_MAX];
  for (uint8_t i = 0; i < n; ++i) {
    pats[i] = rx_buf[3 + i];
    if (pats[i] >= NUM_PATTERNS) return;
  }
  if (n == 1) eng.SelectPattern(pats[0]);
  else        eng.SelectChain(pats, n);
  disp_group = pats[0] / 16;
}

static void handle_sysex(Engine &eng, uint8_t &disp_group) {
  if (rx_len < 2 || rx_buf[0] != SYX_MFR) return;   // not ours (or a bare F0 F7)
  switch (rx_buf[1]) {
    case CMD_REQ_PATTERN:
      if (rx_len >= 3 && rx_buf[2] < NUM_PATTERNS)
        pending_pat_dumps |= (uint32_t)1 << rx_buf[2];
      break;
    case CMD_PUSH_PATTERN: handle_push_pattern(eng); break;
    case CMD_SELECT_PAT:
    case CMD_SELECT_CHAIN: handle_select(eng, disp_group); break;
    case CMD_REQ_TRACK:
      if (rx_len >= 3 && rx_buf[2] < NUM_TRACKS)
        pending_trk_dumps |= (uint8_t)1 << rx_buf[2];
      break;
    case CMD_PUSH_TRACK: handle_push_track(eng); break;
    case CMD_REQ_SETTINGS: midi_send_settings(); break;
    case CMD_SET_SETTINGS:
      // 7D 32 <channel> <clock_source> <out_mode>: apply to RAM now (audible
      // immediately), flag for the deferred flash write, then echo the
      // sanitized result back so the editor reflects any clamping.
      if (rx_len >= 5) {
        g_settings.midi_channel = rx_buf[2];
        g_settings.clock_source = rx_buf[3];
        g_settings.out_mode     = rx_buf[4];
        g_settings.sanitize();
        settings_save_pending = true;
        send_ack(ACK_OK);
        midi_send_settings();
      }
      break;
    default: break;   // unknown command: ignore
  }
}

// ---------------------------------------------------------------------------
// MIDI-IN channel messages (any channel, running status supported)
// ---------------------------------------------------------------------------
// Note On matching the INSTRUMENT_NOTE map = one-shot drum trigger (web-editor
// audition, pads, DAWs); velocity >= 100 also asserts the accent line. Program
// Change selects a pattern (sub*16 + pc), with Bank Select LSB (CC 32) choosing
// the sub = pattern group. Settings.midi_channel gates which channel acts.
static uint8_t chan_status   = 0;     // last Note On / PC / CC status (0 = none)
static uint8_t chan_d1       = 0xFF;  // pending note-on data byte (0xFF = none)
static uint8_t chan_cc_num   = 0xFF;  // pending CC controller number (0xFF = none)
static uint8_t midi_bank_lsb = 0;     // Bank Select LSB (CC 32) = sub / group

// Does this channel-message status pass the configured receive filter?
static inline bool chan_match(uint8_t status) {
  if (g_settings.midi_channel == 0) return true;                  // omni
  return (status & 0x0F) == (uint8_t)(g_settings.midi_channel - 1);
}

// ---------------------------------------------------------------------------
// MIDI-IN System Real-Time (external clock sync)
// ---------------------------------------------------------------------------
// These bytes (0xF8..0xFF) may legally appear ANYWHERE in the stream, even in
// the middle of a SysEx dump or a channel message, so they are handled before
// any of the capture/running-status logic and never disturb it. The counters
// below are drained once per loop pass into the caller's MidiClockIn.
static uint8_t s_clk_pulses = 0;     // 0xF8 ticks accumulated since the last poll
static bool    s_transport  = false; // running: between Start/Continue and Stop
static bool    s_start_edge = false; // Start or Continue seen this poll
static bool    s_stop_edge  = false; // Stop seen this poll

static void rt_byte(uint8_t b) {
  if (b == 0xF8) {                    // Clock: count it for the sequencer, and
    if (s_clk_pulses < 250) ++s_clk_pulses;
    // Forward the incoming clock to MIDI OUT when we're slaving to it (MIDI
    // sync) so downstream gear chases the same clock — or, in THRU mode, as
    // part of the raw soft-thru. Not when we're the internal/DIN master in OUT
    // mode (then we generate our own clock in main.cpp instead).
    if (g_settings.out_mode == OUT_MODE_THRU || g_settings.clock_source == CLK_SRC_MIDI)
      tx_raw(0xF8);                   // non-blocking: forwarding must never stall
                                      // the loop and starve RX of the next clock
    return;
  }
  if (g_settings.out_mode == OUT_MODE_THRU) tx_raw(b);         // soft-thru other realtime
  switch (b) {
    case 0xFA:                        // Start
    case 0xFB:                        // Continue (the 606 has no pause/resume, so
      s_transport  = true;            // this restarts the pattern, same as Start)
      s_start_edge = true;
      s_clk_pulses = 0;               // the downbeat must fire on the first clock
                                      // AFTER Start — drop any clock bytes that
                                      // arrived earlier in this same poll, or the
                                      // first step lands up to a tick early
      break;
    case 0xFC:                        // Stop
      s_transport = false;
      s_stop_edge = true;
      break;
    default: break;                   // 0xF9/0xFD reserved, 0xFE sensing, 0xFF reset: ignore
  }
}

static void chan_byte(Engine &eng, uint8_t &disp_group, uint8_t b) {
  if (b & 0x80) {                                    // new status byte
    const uint8_t type = b & 0xF0;
    chan_status = (type == 0x90 || type == 0xC0 || type == 0xB0) ? b : 0;
    chan_d1     = 0xFF;
    chan_cc_num = 0xFF;
    return;
  }
  if (!chan_status) return;
  const uint8_t type = chan_status & 0xF0;

  if (type == 0xC0) {                                // Program Change: 1 data byte
    if (chan_match(chan_status)) {
      const uint8_t idx = (uint8_t)(midi_bank_lsb * PATS_PER_GROUP + b);
      if (idx < NUM_PATTERNS) {
        eng.SelectPatternSynced(idx);                // sub*16 + pc; lands on the
        disp_group = idx / PATS_PER_GROUP;           // bar even if it arrives a
      }                                              // hair after the bar's tick
    }
    return;                                          // running status: next byte = next PC
  }

  if (type == 0xB0) {                                // Control Change: controller, value
    if (chan_cc_num == 0xFF) { chan_cc_num = b; return; }
    const uint8_t cc = chan_cc_num;
    chan_cc_num = 0xFF;                              // running status: next pair
    // Bank Select sets the sub/group for the next Program Change. CC 0 (MSB) is
    // the bank — the 606 has one, so it is accepted and ignored. CC 32 (LSB) is
    // the sub = pattern group (0 = I, 1 = II), added as a ×16 offset.
    if (cc == 32 && chan_match(chan_status))
      midi_bank_lsb = (b < NUM_GROUPS) ? b : 0;
    return;
  }

  // Note On (0x90): collect note, then velocity
  if (chan_d1 == 0xFF) { chan_d1 = b; return; }
  if (b > 0 && chan_match(chan_status)) {            // velocity 0 = note off
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      if (INSTRUMENT_NOTE[i] == chan_d1) { eng.TriggerNow((uint8_t)1 << i, b >= 100); break; }
  }
  chan_d1 = 0xFF;                                    // running status: next pair
}

static void rx_byte(Engine &eng, uint8_t &disp_group, uint8_t b) {
  if (b >= 0xF8) { rt_byte(b); return; }  // realtime: legal anywhere (even mid-SysEx)
  if (b == 0xF0) {                        // (re)start capture
    rx_active = true;
    rx_len = 0;
    rx_overflow = false;
    chan_status = 0;                      // a system message cancels running status
    return;
  }
  // THRU: soft-thru channel + realtime performance data, but not the editor's
  // SysEx (it stays local). A channel byte arrives either outside a SysEx
  // capture, or as the status byte that aborts one.
  if (!rx_active) {
    if (g_settings.out_mode == OUT_MODE_THRU) tx_raw(b);
    chan_byte(eng, disp_group, b);
    return;
  }
  if (b == 0xF7) {
    rx_active = false;
    if (!rx_overflow) handle_sysex(eng, disp_group);
    return;
  }
  if (b & 0x80) {                      // status aborts the SysEx
    rx_active = false;
    if (g_settings.out_mode == OUT_MODE_THRU) tx_raw(b);
    chan_byte(eng, disp_group, b);
    return;
  }
  if (rx_len < sizeof(rx_buf)) rx_buf[rx_len++] = b;
  else rx_overflow = true;                // too long for us: swallow and discard
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------
void midi_rx_poll(Engine &eng, uint8_t &disp_group, MidiClockIn &mc) {
  s_clk_pulses = 0;                       // reset the per-poll realtime accumulators
  s_start_edge = false;
  s_stop_edge  = false;

  while (Serial1.available() > 0) {
    const uint8_t b = (uint8_t)Serial1.read();
    rx_byte(eng, disp_group, b);
    if (b < 0xF8) last_rx_ms = millis();   // a streaming clock must NOT keep the
                                           // idle-save gate from ever opening
  }

  mc.pulses    = s_clk_pulses;
  mc.transport = s_transport;
  mc.started   = s_start_edge;
  mc.stopped   = s_stop_edge;
}

void midi_tx_service(Engine &eng) {
  // Broadcast the selected pattern (0x1E) whenever it changes while stopped,
  // and on every run->stop transition. The editor re-requests the pattern on
  // each 0x1E, which is how panel edits find their way back to the browser.
  // (While running, the pattern-start anchor from main.cpp covers following.)
  static uint8_t prev_pat     = 0xFF;
  static bool    prev_running = false;
  if (prev_running && !eng.running) {
    send_active_pattern(eng.cur_pat);
    prev_pat = eng.cur_pat;
  } else if (eng.cur_pat != prev_pat) {
    if (!eng.running) send_active_pattern(eng.cur_pat);
    prev_pat = eng.cur_pat;
  }
  prev_running = eng.running;

  service_pending_dumps(eng);
  tx_pump();
}

bool midi_take_save_request(const Engine &eng) {
  if (!save_pending || eng.running) return false;
  if (txq_count > 0 || pending_pat_dumps || pending_trk_dumps) return false;
  if ((uint32_t)(millis() - last_rx_ms) < 250) return false;   // mid-transfer: wait
  save_pending = false;
  return true;
}

bool midi_take_settings_save(const Engine &eng) {
  if (!settings_save_pending || eng.running) return false;     // values are already live in RAM
  settings_save_pending = false;
  return true;
}
