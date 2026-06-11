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

// Logical block ids (must stay < FE_MAX_BLOCKS = 48).
static constexpr uint8_t FB_PATTERN_BASE = 0;                          //  0..31
static constexpr uint8_t FB_TRACK_BASE   = NUM_PATTERNS;               // 32..39
static constexpr uint8_t FB_SETTINGS     = FB_TRACK_BASE + NUM_TRACKS; // 40

static constexpr uint8_t FB_SETTINGS_LEN = 4;  // magic, cur_pat, disp_group, spare

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

// Load patterns + tracks. Power-on selection is deliberately NOT restored:
// the device always wakes on pattern 1, group I, like the stock 606.
inline void load_all(Engine &eng) {
  if (!g_flash_ok) return;
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
    if (g_flash.read(FB_PATTERN_BASE + i, buf, PATTERN_BYTES) == PATTERN_BYTES)
      deserialize_pattern(eng.pattern[i], buf);
  for (uint8_t i = 0; i < NUM_TRACKS; ++i)
    if (g_flash.read(FB_TRACK_BASE + i, buf, TRACK_BYTES) == TRACK_BYTES)
      deserialize_track(eng.track[i], buf);
}

// Write everything whose dirty bit is set, then clear the bits. Each flash page
// write halts the CPU for a few ms, so call only while the sequencer is stopped.
inline void save_dirty(Engine &eng) {
  if (!g_flash_ok) return;
  uint8_t buf[FE_MAX_PAYLOAD];
  for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
    if (!(eng.dirty_pat & ((uint32_t)1 << i))) continue;
    serialize_pattern(eng.pattern[i], buf);
    if (g_flash.write(FB_PATTERN_BASE + i, buf, PATTERN_BYTES))
      eng.dirty_pat &= ~((uint32_t)1 << i);
  }
  for (uint8_t i = 0; i < NUM_TRACKS; ++i) {
    if (!(eng.dirty_trk & (1 << i))) continue;
    serialize_track(eng.track[i], buf);
    if (g_flash.write(FB_TRACK_BASE + i, buf, TRACK_BYTES))
      eng.dirty_trk &= ~(uint8_t)(1 << i);
  }
}
