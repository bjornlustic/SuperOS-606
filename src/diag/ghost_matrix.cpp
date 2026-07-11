// SuperOS-606 — step-LED ghost diagnostic, phase 2: multiplex bisection.
//
// Phase 1 (static single LED, SuperOS port state) was CLEAN on hardware, so
// the quiescent-low data lines are exonerated: the ghost comes from the
// multiplex cycling itself. The ghost-free reference is the D650C ROM, whose
// drive discipline was captured with a host-side trace of the emulator core
// (all its port writes funnel through emu_avr.cpp hook_port). Per 1.8 ms /INT
// tick the ROM services ONE column:
//
//   1. park all selects HIGH (previous column's rows stay driven)
//   2. read the status group
//   3. write the new column's rows, selects still parked
//   4. drop exactly one select LOW; dwell there until the next tick
//
// So: split row/select writes, park between columns with rows kept, 1.8 ms
// dwell, full frame every 7.2 ms. SuperOS's ScanAndDisplay does one combined
// select+rows PORTF write per column at ~124 us. Prior hardware tests showed
// textbook ordering alone and slow dwell alone did NOT fix it, so this image
// bisects the remaining differences in one flash: six drive disciplines, all
// lighting only step 9 (0-based 8), stepped with the TAP key.
//
//   mode 1: hw::ScanAndDisplay itself           expect GHOST (baseline repro)
//   mode 2: exact ROM replica (split writes, park-with-rows, 1.8 ms dwell)
//   mode 3: ROM discipline at SuperOS speed (130 us dwell)   cadence test
//   mode 4: combined single-write columns, 1.8 ms dwell      write-style test
//   mode 5: combined single-write columns, ~124 us, no reads read test
//   mode 6: static PORTF hold (phase-1 control)              expect CLEAN
//
// On boot and after every TAP press, the new mode number k is shown for 1 s
// by statically lighting step k (steps 1-4 = column 0, steps 5-6 = column 1:
// always a single-column static write, which phase 1 proved clean). Then the
// mode's drive runs until the next TAP. After mode 6, TAP wraps to mode 1.
//
// TAP is sampled every 50 ms via a 90 us park of the selects (rows kept);
// that 0.2% disturbance is identical in every mode so it cannot bias the
// comparison. Port state is the real firmware's throughout: hw::Init
// quiescent-low data lines, USB torn down, 16 MHz.
#include <Arduino.h>
#include "pins.h"
#include "hw.h"

static const uint16_t PAYLOAD = (uint16_t)1 << 8;   // step 9 only
static const uint8_t  NUM_MODES = 6;

static uint8_t  s_mode = 1;
static uint32_t s_ind_until = 0;    // mode indicator shown until this millis()
static uint8_t  s_tap_prev = 0;
static uint32_t s_tap_next_ms = 0;

static inline uint8_t rows_of(uint16_t frame, uint8_t col) {
  return (uint8_t)((frame >> (4 * col)) & 0x0F);
}

// Same USB teardown as main.cpp: the Teensy core's usb_init() runs before
// setup(), and an unserviced USB engine at 16 MHz must not be left alive.
static void usb_shutdown_hw() {
  UDIEN  = 0;
  UDCON  = 1;                 // detach
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
}

// TAP = status-group PA1 = PINB bit 1, valid with all selects parked HIGH.
// 90 us settle as in combined.cpp's key-release probe. Rows are kept driven
// during the park (the ROM's own discipline), so all LEDs are simply dark
// for the 90 us window.
static void tap_service() {
  const uint32_t now = millis();
  if ((int32_t)(now - s_tap_next_ms) < 0) return;
  s_tap_next_ms = now + 50;

  const uint8_t save = PORTF;
  PORTF = (uint8_t)((save & 0xF0) | 0x0F);
  delayMicroseconds(90);
  const uint8_t tap = (uint8_t)((PINB >> 1) & 1);
  PORTF = save;

  if (tap && !s_tap_prev) {
    s_mode = (uint8_t)(s_mode % NUM_MODES + 1);
    s_ind_until = now + 1000;
    PORTF = 0x0F;                       // no stale rows across mode changes
  }
  s_tap_prev = tap;
}

// ---- one display iteration per mode ----------------------------------------

static void mode1_scananddisplay() {    // the real thing, ghost baseline
  uint8_t cell[4], st;
  hw::ScanAndDisplay(PAYLOAD, cell, &st);
}

static void mode_rom_replica(uint16_t dwell_us) {   // modes 2 and 3
  static uint8_t col = 0;
  const uint8_t rows = rows_of(PAYLOAD, col);
  PORTF = (uint8_t)((PORTF & 0xF0) | 0x0F);         // park, previous rows kept
  delayMicroseconds(10);                            // ROM reads status here
  PORTF = (uint8_t)((rows << 4) | 0x0F);            // new rows, still parked
  delayMicroseconds(10);
  PORTF = (uint8_t)((rows << 4) | (0x0F & ~(1 << col)));   // one select low
  delayMicroseconds(dwell_us);
  col = (uint8_t)((col + 1) & 3);
}

static void mode_combined_write(uint16_t dwell_us) {   // mode 4
  static uint8_t col = 0;
  PORTF = 0x0F;                                     // park, rows off
  delayMicroseconds(10);
  PORTF = (uint8_t)((rows_of(PAYLOAD, col) << 4) | (0x0F & ~(1 << col)));
  delayMicroseconds(dwell_us);
  col = (uint8_t)((col + 1) & 3);
}

static void mode5_scan_shaped() {       // ScanAndDisplay's shape, no pin reads
  PORTF = 0x0F;                         // status phase: 3 + 4*12 us, rows off
  delayMicroseconds(51);
  for (uint8_t col = 0; col < 4; ++col) {
    PORTF = (uint8_t)((rows_of(PAYLOAD, col) << 4) | (0x0F & ~(1 << col)));
    delayMicroseconds(124);             // settle + 8 reads + col_us 25
  }
}

void setup() {
  usb_shutdown_hw();
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);   // as in main.cpp: no floating MIDI RX
  hw::Init();                           // SuperOS port state, PORTF = 0x0F
  s_ind_until = millis() + 1000;        // announce mode 1 on boot
}

void loop() {
  tap_service();

  if ((int32_t)(millis() - s_ind_until) < 0) {      // static mode-number LED
    const uint8_t n = (uint8_t)(s_mode - 1);        // step index 0..5
    hw::LightCell((uint8_t)(n >> 2), (uint8_t)(n & 3));
    return;
  }

  switch (s_mode) {
    case 1: mode1_scananddisplay();     break;
    case 2: mode_rom_replica(1780);     break;      // 1.8 ms burst, ROM cadence
    case 3: mode_rom_replica(130);      break;
    case 4: mode_combined_write(1790);  break;
    case 5: mode5_scan_shaped();        break;
    default: hw::LightCell(2, 0);       break;      // mode 6: static step 9
  }
}
