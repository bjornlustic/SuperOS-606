// SuperOS-606 — low-level matrix driver
//

#pragma once
#include <Arduino.h>
#include <string.h>
#include "pins.h"

namespace hw {

inline void Init() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(SELECT_PINS[i], OUTPUT);
    pinMode(LED_PINS[i], OUTPUT);
    pinMode(SW_PINS[i], INPUT);
    pinMode(STATUS_PINS[i], INPUT);
  }
  for (uint8_t i = 0; i < sizeof(QUIESCENT_LOW_PINS); ++i) {
    pinMode(QUIESCENT_LOW_PINS[i], OUTPUT);
    digitalWrite(QUIESCENT_LOW_PINS[i], LOW);
  }
  PORTF = 0x0F; // all selects high (inactive), all LEDs off
}

// Light exactly one step-LED matrix cell. sel/led are 0..3. Pass led >= 4 for none.
inline void LightCell(uint8_t sel, uint8_t led) {
  uint8_t f = 0x0F;            // selects high, LEDs low
  if (led < 4) {
    f &= ~(1 << sel);          // pull this scan column LOW (active)
    f |= (1 << (4 + led));     // drive this LED row HIGH
  }
  PORTF = f;
}

inline void AllOff() { PORTF = 0x0F; }

// Read the whole switch matrix.
//   cell[s] (s = select 0..3): bit0-3 = PB0-3, bit4-7 = PA0-3 while select s is LOW
//   *status                  : bit0-3 = PA0-3 while ALL selects are HIGH
inline void ScanMatrix(uint8_t cell[4], uint8_t *status) {
  PORTF = 0x0F;                       // all selects high
  delayMicroseconds(3);
  uint8_t st = 0;
  for (uint8_t j = 0; j < 4; ++j) st |= (digitalRead(STATUS_PINS[j]) << j);
  *status = st;

  for (uint8_t s = 0; s < 4; ++s) {
    PORTF = (uint8_t)(0x0F & ~(1 << s)); // pull select s LOW
    delayMicroseconds(3);
    uint8_t v = 0;
    for (uint8_t j = 0; j < 4; ++j) v |= (digitalRead(SW_PINS[j]) << j);
    for (uint8_t j = 0; j < 4; ++j) v |= (digitalRead(STATUS_PINS[j]) << (4 + j));
    cell[s] = v;
  }
  PORTF = 0x0F;
}

// One combined scan + display pass (~1 ms): reads the matrix and status like
// ScanMatrix, while also multiplexing a 16-bit step-LED frame (bit n = LED n)
// one column at a time. Each column gets `col_us` of LED dwell, so the frame is
// shown at ~25% duty. This is the main loop's heartbeat: calling it
// continuously keeps the LEDs lit and the switches sampled at ~1 kHz.
inline void ScanAndDisplay(uint16_t frame, uint8_t cell[4], uint8_t *status,
                           uint16_t col_us = 150) {
  PORTF = 0x0F;                       // all selects high -> status group valid
  delayMicroseconds(3);
  uint8_t st = 0;
  for (uint8_t j = 0; j < 4; ++j) st |= (digitalRead(STATUS_PINS[j]) << j);
  *status = st;

  for (uint8_t s = 0; s < 4; ++s) {
    const uint8_t leds = (uint8_t)((frame >> (4 * s)) & 0x0F);
    PORTF = (uint8_t)((0x0F & ~(1 << s)) | (leds << 4));  // select low + LED rows
    delayMicroseconds(3);
    uint8_t v = 0;
    for (uint8_t j = 0; j < 4; ++j) v |= (digitalRead(SW_PINS[j]) << j);
    for (uint8_t j = 0; j < 4; ++j) v |= (digitalRead(STATUS_PINS[j]) << (4 + j));
    cell[s] = v;
    delayMicroseconds(col_us);                            // LED dwell
  }
  PORTF = 0x0F;
}

} // namespace hw
