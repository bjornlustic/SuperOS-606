// SuperOS-606 — TR-606 firmware for the µPD650 CPU-replacement board (AT90USB1286)
//
// pins.h — the µPD650 CPU pinout as wired on the replacement board, annotated with
//          TR-606 port semantics (TR-606 service manual p.2, "µPD650C functional
//          description").
//
//   Port  Dir  TR-606 use                                  µPD650 pins
//   ----  ---  ------------------------------------------  -----------
//   PH0-3 OUT  scan-select to switches & status            26-29
//   PG0-3 OUT  drive signals to the 16 STEP LEDs           22-25
//   PA0-3 IN   switch-scan inputs + status                 33-36
//                (status, all PH high: tempo-clk, run/stop, tap)
//   PB0-3 IN   inputs from STEP switches / rhythm-select   37-40
//   PC0-3 I/O  RAM (µPD444C) data bus, via 3.3k            2-5
//   PD0-3 OUT  rhythm number / RAM address                 8-11
//   PF0-3 OUT  step number / instrument data (LT,SD,BD,AC) 16-19
//   PE0-3 OUT  I/II + memory-bank select (CH,OH,CY,HT)     12-15
//   PI0   OUT  RAM WE                                       30
//   PI1   OUT  RAM CE                                       31
//   PI2   OUT  trigger-pulse (INSTRUMENT) output            32
//
#pragma once
#include <Arduino.h>

// Teensy++ 2.0 Arduino pin numbers for each µPD650 pin (board wiring).
enum CpuPin : uint8_t {
  MIDI_IN_PIN  = 2,   // Serial1 RX
  MIDI_OUT_PIN = 3,   // Serial1 TX

  // PC0-3 : RAM data bus
  PC0_PIN = 4, PC1_PIN = 5, PC2_PIN = 6, PC3_PIN = 7,

  // PI : RAM CE + trigger-pulse out
  PI1_PIN = 8,   // RAM CE
  PI2_PIN = 9,   // trigger-pulse output

  // PD0-3 + PF0-3 : RAM address + instrument data
  PD0_PIN = 10, PD1_PIN = 11, PD2_PIN = 12, PD3_PIN = 13,
  PF0_PIN = 14, PF1_PIN = 15, PF2_PIN = 16, PF3_PIN = 17,

  // PE0-3 : I/II + bank select
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

// PH (select) + PG (LED) are the low + high nibbles of Teensy PORTF, so the whole
// LED/scan matrix can be driven with a single byte write to PORTF:
//   low nibble  = PH0-3  (drive a bit LOW to activate that scan column)
//   high nibble = PG0-3  (drive a bit HIGH to light that LED row)
// PORTF = 0x0F  -> all selects high (inactive), all LEDs off.

static const uint8_t SELECT_PINS[4] = { PH0_PIN, PH1_PIN, PH2_PIN, PH3_PIN }; // scan columns
static const uint8_t LED_PINS[4]    = { PG0_PIN, PG1_PIN, PG2_PIN, PG3_PIN }; // step-LED rows
static const uint8_t SW_PINS[4]     = { PB0_PIN, PB1_PIN, PB2_PIN, PB3_PIN }; // step-switch inputs
static const uint8_t STATUS_PINS[4] = { PA0_PIN, PA1_PIN, PA2_PIN, PA3_PIN }; // status/scan inputs

// Data / trigger lines. Held as OUTPUT-LOW at idle so the 606's voice-trigger logic
// and RAM bus stay quiescent (trigger pulse PI2 idle LOW => no voices fire). The
// firmware drives these to actually trigger drums.
static const uint8_t QUIESCENT_LOW_PINS[] = {
  PC0_PIN, PC1_PIN, PC2_PIN, PC3_PIN,
  PD0_PIN, PD1_PIN, PD2_PIN, PD3_PIN,
  PF0_PIN, PF1_PIN, PF2_PIN, PF3_PIN,
  PE0_PIN, PE1_PIN, PE2_PIN, PE3_PIN,
  PI1_PIN, PI2_PIN,
};
