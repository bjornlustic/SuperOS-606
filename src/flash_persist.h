// SuperOS-606 — device glue between the sequencer and the flash block store
// (flash_eeprom.h). Provides the page program/read hooks, the single
// FlashEeprom instance, the logical block-id map, and pattern/track/settings
// load+save helpers.
//
// Writes go through the boot-section SPM service (flash_store.h). The service
// is installed once per board via service-install.syx (env:flash-service); if
// it is absent, everything still runs — patterns just don't survive power-off.
#pragma once
#include <Arduino.h>
#include "flash_store.h"
#include "flash_eeprom.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"

// Logical block ids (must stay < FE_MAX_BLOCKS = 48).
static constexpr uint8_t FB_PATTERN_BASE = 0;                          //  0..31
static constexpr uint8_t FB_TRACK_BASE   = NUM_PATTERNS;               // 32..39
static constexpr uint8_t FB_SETTINGS     = FB_TRACK_BASE + NUM_TRACKS; // 40

// FB_SETTINGS payload: magic, midi_channel, clock_source, out_mode (see
// settings.h). The magic gates a blank/older block so a fresh device falls back
// to the struct defaults instead of garbage.
static constexpr uint8_t FB_SETTINGS_LEN  = 4;
static constexpr uint8_t SETTINGS_MAGIC   = 0x96;

// Page hooks: program via the boot SPM service, read via far program-memory reads.
inline uint8_t fe_dev_program(uint16_t abs_page, const uint8_t *buf) {
  return flash_write_page(abs_page, buf);
}
inline void fe_dev_read(uint16_t abs_page, uint8_t *buf) {
  uint32_t a = (uint32_t)abs_page << 8;
  for (uint16_t i = 0; i < FE_PAGE; ++i) buf[i] = pgm_read_byte_far(a + i);
}

FlashEeprom g_flash;
static bool g_flash_ok = false;

// Mount the arena (formats if blank). False if the SPM service is missing —
// the app then runs RAM-only.
inline bool flash_persist_begin() {
  if (!flash_service_present()) return false;
  g_flash_ok = g_flash.begin(fe_dev_program, fe_dev_read);
  return g_flash_ok;
}

// Flash-resident pattern IO hooks (registered with the Engine). Patterns live in
// the flash block store; only the active one is cached in RAM. Read returns false
// on a blank/older block so the cache falls back to an empty pattern.
inline bool flash_pat_read(uint8_t idx, Pattern &out) {
  if (!g_flash_ok) return false;
  uint8_t buf[FE_MAX_PAYLOAD];
  // Cap at the full buffer, not PATTERN_BYTES: read() clamps to the cap, so a
  // longer record from an OLD format would otherwise clamp to exactly
  // PATTERN_BYTES and load truncated garbage instead of being rejected.
  if (g_flash.read(FB_PATTERN_BASE + idx, buf, sizeof(buf)) != PATTERN_BYTES) return false;
  deserialize_pattern(out, buf);
  return true;
}
inline bool flash_pat_write(uint8_t idx, const Pattern &in) {
  if (!g_flash_ok) return false;
  uint8_t buf[FE_MAX_PAYLOAD];
  serialize_pattern(in, buf);
  return g_flash.write(FB_PATTERN_BASE + idx, buf, PATTERN_BYTES);
}

// Register the pattern hooks and load tracks. Patterns are loaded lazily by the
// engine cache on first access. Power-on selection is deliberately NOT restored:
// the device always wakes on pattern 1, group I, like the stock 606.
inline void load_all(Engine &eng) {
  eng.SetPatIO(flash_pat_read, flash_pat_write);   // hooks self-guard on g_flash_ok
  if (!g_flash_ok) return;
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_TRACKS; ++i)
    if (g_flash.read(FB_TRACK_BASE + i, buf, TRACK_BYTES) == TRACK_BYTES)
      deserialize_track(eng.track[i], buf);
}

// Load the global settings, falling back to the struct defaults when the block
// is blank or from an older firmware (magic mismatch). Call once at boot.
inline void load_settings(Settings &s) {
  if (!g_flash_ok) return;
  uint8_t buf[FB_SETTINGS_LEN];
  if (g_flash.read(FB_SETTINGS, buf, FB_SETTINGS_LEN) == FB_SETTINGS_LEN &&
      buf[0] == SETTINGS_MAGIC) {
    s.midi_channel = buf[1];
    s.clock_source = buf[2];
    s.out_mode     = buf[3];
    s.sanitize();
  }
}

// Persist the global settings (one tiny flash write). Call only while stopped.
inline void save_settings(const Settings &s) {
  if (!g_flash_ok) return;
  const uint8_t buf[FB_SETTINGS_LEN] = {
    SETTINGS_MAGIC, s.midi_channel, s.clock_source, s.out_mode };
  g_flash.write(FB_SETTINGS, buf, FB_SETTINGS_LEN);
}

// Write everything whose dirty bit is set, then clear the bits. Each flash page
// write halts the CPU for a few ms, so call only while the sequencer is stopped.
inline void save_dirty(Engine &eng) {
  if (!g_flash_ok) return;
  eng.Flush();                  // persist the dirty pattern cache + any deferred write
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_TRACKS; ++i) {
    if (!(eng.dirty_trk & (1 << i))) continue;
    serialize_track(eng.track[i], buf);
    if (g_flash.write(FB_TRACK_BASE + i, buf, TRACK_BYTES))
      eng.dirty_trk &= ~(uint8_t)(1 << i);
  }
}
