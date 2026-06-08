// SuperOS-606 — low-level matrix driver (diagnostic stage)
//
// Only touches the parts of the µPD650 interface that are SAFE regardless of the
// exact 606 matrix ordering: it drives the PH (scan) + PG (LED) outputs and reads
// the PB/PA inputs. All data/trigger lines are pinned OUTPUT-LOW so no drum voices
// fire and the RAM bus stays quiet. This file will grow into the full 606 driver
// (trigger output, etc.) once the matrix map is confirmed on the real unit.
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

}
