// SuperOS-606 — TR-606 firmware for the Teensy++ 2.0 (AT90USB1286)
//
// pins.h — the µPD650 CPU pinout as wired to the Teensy++ 2.0 that replaces it,
//          annotated with TR-606 port semantics.
//
// The Teensy-pin <-> µPD650-pin wiring is a fixed property of the replacement
// board. The TR-606 use of each port is from the service manual p.2 ("µPD650C
// functional description"); see the table below.
//
//   Port  Dir  TR-606 use                                  µPD650 pins
//   ----  ---  ------------------------------------------  -----------
//   PH0-3 OUT  scan-select to switches & status            26-29
//   PG0-3 OUT  drive signals to the 16 STEP LEDs           22-25
//   PA0-3 IN   switch-scan inputs + status                 33-36
//                (status, all PH high: tempo-clk, run/stop, tap)
//   PB0-3 IN   inputs from STEP switches / rhythm-select   37-40
//   PC0-3 I/O  RAM (µPD444C) data bus, via 3.3k  (NOT LEDs on 606)  2-5
//   PD0-3 OUT  rhythm number / RAM address                 8-11
//   PF0-3 OUT  step number / instrument data (LT,SD,BD,AC) 16-19
//   PE0-3 OUT  I/II + memory-bank select (CH,OH,CY,HT)     12-15
//   PI0   OUT  RAM WE                                       30
//   PI1   OUT  RAM CE                                       31
//   PI2   OUT  trigger-pulse (INSTRUMENT) output            32
//
#pragma once
#include <Arduino.h>

// Teensy++ 2.0 Arduino pin numbers for each µPD650 pin (replacement-board wiring).
enum TppPin : uint8_t {
  MIDI_IN_PIN  = 2,   // Serial1 RX
  MIDI_OUT_PIN = 3,   // Serial1 TX

  // PC0-3 : 606 = RAM data bus
  PC0_PIN = 4, PC1_PIN = 5, PC2_PIN = 6, PC3_PIN = 7,

  // PI : 606 = RAM CE + trigger-pulse out
  PI1_PIN = 8,   // 606 RAM CE
  PI2_PIN = 9,   // 606 trigger-pulse output

  // PD0-3 + PF0-3 : 606 = RAM address + instrument data
  //   PF0-3 = the four INSTRUMENT-DATA bits (service manual p.2): set the bit
  //   high, then pulse COMMON TRIG (PI2) to fire that voice / accent.
  PD0_PIN = 10, PD1_PIN = 11, PD2_PIN = 12, PD3_PIN = 13,
  PF0_PIN = 14,  // 606 = LT instrument data   (µPD650 pin 16)
  PF1_PIN = 15,  // 606 = SD instrument data   (µPD650 pin 17)
  PF2_PIN = 16,  // 606 = BD instrument data   (µPD650 pin 18)
  PF3_PIN = 17,  // 606 = AC (ACCENT) data     (µPD650 pin 19)

  // PE0-3 : 606 = I/II + bank select
  PE2_PIN = 0, PE3_PIN = 1, PE0_PIN = 18, PE1_PIN = 19,

  // PB0-3 : step-switch inputs
  PB0_PIN = 24, PB1_PIN = 25, PB2_PIN = 26, PB3_PIN = 27,

  // PA0-3 : switch-scan + status inputs
  PA0_PIN = 20, PA1_PIN = 21, PA2_PIN = 22, PA3_PIN = 23,

  // PH0-3 : scan-select outputs  (Teensy PORTF low nibble, pins 38-41)
  PH0_PIN = 38, PH1_PIN = 39, PH2_PIN = 40, PH3_PIN = 41,

  // PG0-3 : step-LED drive outputs (Teensy PORTF high nibble, pins 42-45)
  PG0_PIN = 42, PG1_PIN = 43, PG2_PIN = 44, PG3_PIN = 45,
};

// ACCENT is just the fourth instrument-data bit: µPD650 PF3 = socket pin 19 =
// Teensy pin 17 (= PF3_PIN). Per the service manual p.2 functional table, PF0-3
// carry LT/SD/BD/AC and "need COMMON TRIG to trigger" — so AC is driven HIGH on
// accented steps and pulsed by PI2, exactly like the seven voice bits. No board
// mod or jumper is involved; the pin is a normal, wired CPU output.
static constexpr uint8_t AC_PIN = PF3_PIN;

// PH (select) + PG (LED) are the low + high nibbles of Teensy PORTF, so the whole
// LED/scan matrix can be driven with a single byte write to PORTF:
//   low nibble  = PH0-3  (drive a bit LOW to activate that scan column)
//   high nibble = PG0-3  (drive a bit HIGH to light that LED row)
// PORTF = 0x0F  -> all selects high (inactive), all LEDs off.

static const uint8_t SELECT_PINS[4] = { PH0_PIN, PH1_PIN, PH2_PIN, PH3_PIN }; // scan columns
static const uint8_t LED_PINS[4]    = { PG0_PIN, PG1_PIN, PG2_PIN, PG3_PIN }; // step-LED rows
static const uint8_t SW_PINS[4]     = { PB0_PIN, PB1_PIN, PB2_PIN, PB3_PIN }; // step-switch inputs
static const uint8_t STATUS_PINS[4] = { PA0_PIN, PA1_PIN, PA2_PIN, PA3_PIN }; // status/scan inputs

// Data / trigger lines. During the diagnostic these are held as OUTPUT-LOW so the
// 606's voice-trigger logic and RAM bus stay quiescent (trigger pulse PI2 idle LOW
// => no voices fire). The real firmware will drive these to actually trigger drums.
static const uint8_t QUIESCENT_LOW_PINS[] = {
  PC0_PIN, PC1_PIN, PC2_PIN, PC3_PIN,
  PD0_PIN, PD1_PIN, PD2_PIN, PD3_PIN,
  PF0_PIN, PF1_PIN, PF2_PIN, PF3_PIN,
  PE0_PIN, PE1_PIN, PE2_PIN, PE3_PIN,
  PI1_PIN, PI2_PIN,
};
