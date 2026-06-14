// SuperOS-606 — MIDI IN/OUT plumbing for the web pattern editor
// (implementation in midi.cpp; the editor lives in tools/web-pattern-edit/)
//
// SysEx envelope: F0 7D <cmd> ... F7 — the same manufacturer id the bootloader
// uses. Command spaces don't collide: the bootloader owns 0x01/0x02 (and only
// runs at boot), the app owns 0x10+.
//
// Patterns (18 bytes, see pattern.h) and tracks (65 bytes) travel 7-bit packed
// (bootloader scheme: 7 data bytes -> 1 MSB byte + 7 low-7-bit bytes) with an
// XOR checksum of the raw bytes sent as two 7-bit values (lo7, hi-bit).
//
//   editor -> 606
//     F0 7D 10 <pat 0-31> F7                              request pattern
//     F0 7D 12 <pat> <xor_lo> <xor_hi> <21B packed> F7    push pattern
//     F0 7D 1A <n 1-16> <p0..pn-1> F7                     select range chain
//     F0 7D 1D <pat 0-31> F7                              select pattern
//     F0 7D 24 <trk 0-7> F7                               request track
//     F0 7D 26 <trk> <xor_lo> <xor_hi> <75B packed> F7    push track
//   606 -> editor
//     F0 7D 11 <pat> <xor_lo> <xor_hi> <21B packed> F7    pattern dump
//     F0 7D 14 <status> F7                                push ack (0 = ok)
//     F0 7D 15 <pat 0-31> F7                              pattern-start anchor (running)
//     F0 7D 16 <pat> <inst 0-7> <step 0-15> <on 0|1> F7   panel step edit
//     F0 7D 18 <pat> <len 1-16> F7                        panel length set
//     F0 7D 19 <pat> <scale 0-3> F7                       panel scale set
//     F0 7D 1E <pat 0-31> F7                              selected pattern (stopped / on stop)
//     F0 7D 25 <trk> <xor_lo> <xor_hi> <75B packed> F7    track dump
//
// Selections (0x1D / 0x1A) behave exactly like panel presses in PATTERN PLAY:
// immediate when stopped; while running they queue and take over when the
// playing pattern — or the whole active chain — finishes. The displayed
// pattern group follows the selection so the step LEDs show it.
//
// MIDI-IN Note Ons (any channel) whose note matches the INSTRUMENT_NOTE map
// fire that drum voice with a one-shot trigger pulse — this is how the web
// editor auditions steps, and it makes the 606 playable from a DAW or pads.
// Velocity >= 100 asserts the accent line for the hit. MIDI Program Change
// 0-31 (any channel) selects that pattern, same semantics as 0x1D.
//
// MIDI sync (external clock IN): System Real-Time messages slave the sequencer
// to an external MIDI clock. While clock (0xF8) is arriving the sequencer steps
// off it at 24 PPQN instead of the 606's own tempo oscillator, and Start/Stop/
// Continue (0xFA/0xFC/0xFB) run and stop it; it falls back to the internal
// TEMPO-knob / DIN clock once the external clock stops. Auto-detected — any
// incoming clock takes over (there is no spare panel control to gate it). The
// received clock is forwarded 1:1 to MIDI OUT so downstream gear stays in sync.
//
// Pushed patterns/tracks land in RAM immediately (audible on the next step)
// and are flagged dirty; flash persistence happens through the normal
// save-on-stop path plus an idle save the main loop performs when
// midi_take_save_request() reports one (never mid-transfer: each SPM page
// write halts the CPU for a few ms and would drop incoming MIDI bytes).
#pragma once
#include <Arduino.h>

class Engine;

/// External MIDI clock-in state, filled by midi_rx_poll() each loop pass so
/// main.cpp can clock the sequencer from an external MIDI source.
struct MidiClockIn {
  uint8_t pulses    = 0;      ///< 0xF8 clock ticks received since the last poll
  bool    transport = false;  ///< running: between a MIDI Start/Continue and a Stop
  bool    started   = false;  ///< a Start/Continue arrived this poll (resync to step 0)
  bool    stopped   = false;  ///< a Stop arrived this poll
};

/// Drain MIDI IN for this loop pass: SysEx (pattern/track transfer, remote
/// select), channel messages (note triggers, program change), AND System
/// Real-Time clock/transport. The realtime state is reported in `mc` so the
/// main loop can step the sequencer off an external clock; received clock is
/// forwarded 1:1 to MIDI OUT here. Call near the top of the loop, before
/// transport/clock processing. `disp_group` follows remote selections so the
/// panel LEDs match.
void midi_rx_poll(Engine &eng, uint8_t &disp_group, MidiClockIn &mc);

/// Outgoing-side housekeeping: broadcast 0x1E on stopped selection changes and
/// on every run->stop, service queued pattern/track dumps, and pump the TX
/// queue. Call once per loop pass, AFTER transport/mode processing (it reads
/// eng.running / eng.cur_pat, which those steps update).
void midi_tx_service(Engine &eng);

/// Queue one complete MIDI message for transmit. Messages go out atomically
/// (never interleaved with each other) and only as fast as the UART can take
/// them, so queuing never blocks the ~1 ms loop. Realtime bytes (clock/start/
/// stop) should keep bypassing the queue — the MIDI spec lets them interleave
/// anywhere, and they must not wait behind a queued pattern dump.
/// Returns false if the queue is full (the message is dropped).
bool midi_tx_msg(const uint8_t *msg, uint8_t len);

/// Broadcast the pattern-start anchor (SysEx 0x15). Call at every pattern
/// start (step 0 fire) while running; the editor re-anchors its clock-derived
/// playhead on it and follows pattern/chain/track switches.
void midi_send_step_position(uint8_t pat);

/// Panel-edit broadcasts (SysEx 0x16 / 0x18 / 0x19): call after every step
/// toggle / tap write / chase-delete, length set, and scale arm so the web
/// editor mirrors writes made on the 606 live.
void midi_send_step_update(uint8_t pat, uint8_t inst, uint8_t step, bool on);
void midi_send_length_update(uint8_t pat, uint8_t len);
void midi_send_scale_update(uint8_t pat, uint8_t scale);

/// Queue a full pattern dump (SysEx 0x11) toward the editor — used after
/// PATTERN CLEAR, where per-step broadcasts would take 16 messages.
void midi_send_pattern_dump(uint8_t pat);

/// True exactly once when pushed data is ready to persist: something was
/// pushed, the sequencer is stopped, MIDI has been idle for a moment and the
/// TX queue is empty. The caller must then call save_dirty(eng) — the request
/// is cleared by this call.
bool midi_take_save_request(const Engine &eng);
