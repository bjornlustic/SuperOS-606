// combined.cpp -- entry dispatch for the combined SuperOS-606 + D650C build.
// Empty TU in non-combined builds. Ported from SuperOS-303's combined.cpp
// (same MCU, same bootloader scheme, same USB-PLL failure mode).
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "pins.h"
#include "hw.h"
#include "combined.h"

// Pre-main boot guard. The Teensy core's usb_init() runs BEFORE setup() and
// busy-waits forever on USB PLL lock. If the PLL misses lock -- seen on the
// 303 after a watchdog reboot landed while the previous firmware's USB/PLL
// was still running -- the unit wedges dead until power-off. Arm a 2 s
// watchdog from .init3 so ANY pre-setup hang retries the boot. setup()
// replaces it with the permanent 8 s loop watchdog.
extern "C" __attribute__((naked, used, section(".init3")))
void combined_boot_guard(void) {
  MCUSR = 0;
  wdt_enable(WDTO_2S);
}

// main.cpp and d650/emu_avr.cpp rename their entry points to these.
void superos_setup(); void superos_loop(); void superos_pcint();
void emu_setup();     void emu_loop();     void emu_pcint();

static uint8_t g_fw = FW_SUPEROS;

// Both firmwares use PCINT0 on the PA3 status line; dispatch by active side.
ISR(PCINT0_vect) {
  if (g_fw == FW_D650) emu_pcint(); else superos_pcint();
}

// Power-on escape: CLEAR + FUNCTION + PATTERN GROUP held while switching on
// forces SuperOS. One direct matrix probe of row 3 (PH3 low, PA0/1/2 = the
// three buttons), before either firmware starts.
static bool escape_held() {
  hw::Init();                       // selects high, inputs floating-defined
  PORTF = 0x0F & ~(1 << 3);         // select row 3
  delayMicroseconds(90);            // PA lines settle slowly (see hw.h note)
  const uint8_t pa = PINB & 0x07;   // PA0=CLEAR, PA1=FUNCTION, PA2=GROUP
  PORTF = 0x0F;
  return pa == 0x07;
}

void setup() {
  MCUSR = 0;
  // Permanent 8 s freeze-recovery watchdog, petted every loop() pass (root
  // cause on the 303: intermittent post-swap USB freeze below our code).
  wdt_enable(WDTO_8S);
  const uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_D650) ? FW_D650 : FW_SUPEROS;
  if (g_fw == FW_D650 && escape_held()) {
    g_fw = FW_SUPEROS;
    eeprom_update_byte(EE_FW_SELECT, FW_SUPEROS);
  }
  if (g_fw == FW_D650) emu_setup(); else superos_setup();
}

void loop() {
  wdt_reset();
  if (g_fw == FW_D650) emu_loop(); else superos_loop();
}

void combined_switch_firmware(uint8_t fw) {
  wdt_reset();                       // fresh 8 s budget for the lockout below
  eeprom_update_byte(EE_FW_SELECT, fw);
  // Input lockout before the reboot: a key still held through the reset can
  // land in the bootloader's stay-resident window (TAP = PINB bit1 sampled
  // ~40 ms after reset; the diode-less matrix can ghost held keys onto it).
  // Wait until every momentary key reads released for 250 ms straight:
  // TAP (status PA1), the 16 step keys (PB x 4 rows), and the row-3 buttons
  // (CLEAR/FUNCTION/GROUP). Rotary levels and the RUN FF level are ignored.
  // Hard 4 s timeout so a stuck line can't block the switch forever.
  {
    const uint32_t t0 = millis();
    uint32_t idle_since = millis();
    while (millis() - t0 < 4000) {
      bool any = false;
      PORTF = 0x0F; delayMicroseconds(90);
      if (PINB & 0x02) any = true;                    // TAP
      for (uint8_t s = 0; s < 4; ++s) {
        PORTF = (uint8_t)(0x0F & ~(1 << s)); delayMicroseconds(90);
        if (PINB & 0xF0) any = true;                  // step keys
        if (s == 3 && (PINB & 0x07)) any = true;      // CLEAR/FUNCTION/GROUP
      }
      PORTF = 0x0F;
      const uint32_t now = millis();
      if (any) idle_since = now;
      else if (now - idle_since >= 250) break;
      delay(2);
    }
  }
  // Shut USB and its PLL down cleanly before the reset so the next boot's
  // usb_init() starts the PLL cold (the 303's PLL-lock-hang trigger).
  cli();
  UDIEN = 0;
  UDCON |= (1 << DETACH);
  _delay_ms(30);
  USBCON = 0;
  PLLCSR = 0;
  wdt_enable(WDTO_15MS);             // hardware reset via watchdog
  for (;;) {}
}
#endif
