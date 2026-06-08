// SuperOS-606 — VOICE BUTTON TEST (power-on firmware)
//
// This is the firmware that runs the moment the 606 powers on. It maps the seven
// drum voices onto the first seven STEP buttons so you can play each instrument by
// hand and confirm — by ear — that every voice triggers:
//
//   Button 1 = BD   Button 2 = SD   Button 3 = LT   Button 4 = HT
//   Button 5 = CH   Button 6 = OH   Button 7 = CY
//
// Press a step button and that voice fires once (one hit per press); its step LED
// lights while you hold it. A MIDI Note-On (note 36 + voice index) is also sent so
// the hit can be confirmed on a MIDI monitor.
//
// Firing uses the 606 trigger-output method: set the voice's instrument-data bit
// HIGH, pulse the COMMON TRIGGER (PI2) HIGH for ~1.5 ms with the data held stable,
// then clear.

#include <Arduino.h>
#include "pins.h"
#include "hw.h"
#include "controls.h"  // confirmed panel decode: step(n) reads panel button n+1

static constexpr uint16_t TRIG_US       = 1500;  // common-trigger pulse width
static constexpr uint8_t  MIDI_NOTE_0   = 36;    // BD = note 36 (C1), then +index
static constexpr uint8_t  SCAN_DELAY_MS = 4;     // loop period (also a light debounce)

static inline void midiNoteOn(uint8_t n, uint8_t v) { Serial1.write(0x90); Serial1.write((uint8_t)(n & 0x7F)); Serial1.write((uint8_t)(v & 0x7F)); }
static inline void midiNoteOff(uint8_t n)           { Serial1.write(0x80); Serial1.write((uint8_t)(n & 0x7F)); Serial1.write((uint8_t)0); }
static inline void midiCC(uint8_t c, uint8_t v)     { Serial1.write(0xB0); Serial1.write((uint8_t)(c & 0x7F)); Serial1.write((uint8_t)(v & 0x7F)); }

// The 7 voices, in button order. Button i (0-based) -> VOICES[i], and its panel
// step LED is also cell i. Pins are the instrument-data lines confirmed audible.
struct Voice { const char *name; uint8_t pin; };
static const Voice VOICES[] = {
  {"BD", PF2_PIN},   // button 1
  {"SD", PF1_PIN},   // button 2
  {"LT", PF0_PIN},   // button 3
  {"HT", PE3_PIN},   // button 4
  {"CH", PE0_PIN},   // button 5
  {"OH", PE1_PIN},   // button 6
  {"CY", PE2_PIN},   // button 7
};
static const uint8_t NUM_VOICES = sizeof(VOICES) / sizeof(VOICES[0]);

// Every instrument-data line — idled LOW so only the fired voice is selected.
static const uint8_t DATA_PINS[] = {
  PF0_PIN, PF1_PIN, PF2_PIN, PF3_PIN, PE0_PIN, PE1_PIN, PE2_PIN, PE3_PIN,
};

static void idleAllData() {
  for (uint8_t i = 0; i < sizeof(DATA_PINS); ++i) digitalWrite(DATA_PINS[i], LOW);
  digitalWrite(PI2_PIN, LOW);
}

// Fire one voice: select only its data bit, pulse the common trigger, then clear.
static void fire(const Voice &v) {
  idleAllData();
  digitalWrite(v.pin, HIGH);          // select this voice
  digitalWrite(PI2_PIN, HIGH);        // common-trigger pulse, data held stable
  delayMicroseconds(TRIG_US);
  digitalWrite(PI2_PIN, LOW);
  digitalWrite(v.pin, LOW);
}

void setup() {
  Serial1.begin(31250);   // MIDI baud
  hw::Init();             // matrix I/O + all data/trigger lines idled OUTPUT-LOW
  idleAllData();
  delay(150);

  // boot signature: three quick CC#119 pulses so you can confirm MIDI OUT is alive
  for (uint8_t i = 0; i < 3; ++i) { midiCC(0x77, 0x7F); delay(70); midiCC(0x77, 0); delay(70); }
}

void loop() {
  static bool prev[NUM_VOICES] = { false };   // last-seen press state, for edge detect

  Controls in;
  in.Scan();

  int8_t held = -1;                           // lowest button held now (for LED feedback)
  for (uint8_t i = 0; i < NUM_VOICES; ++i) {
    const bool pressed = in.step(i);          // step switch i == panel button (i + 1)
    if (pressed && !prev[i]) {                // rising edge: play the voice once
      midiNoteOn(MIDI_NOTE_0 + i, 100);
      fire(VOICES[i]);
    } else if (!pressed && prev[i]) {         // release
      midiNoteOff(MIDI_NOTE_0 + i);
    }
    prev[i] = pressed;
    if (pressed && held < 0) held = i;
  }

  // echo your touch on the panel: light the held button's step LED, else all off
  if (held >= 0) hw::LightStep(held); else hw::AllOff();

  delay(SCAN_DELAY_MS);
}
