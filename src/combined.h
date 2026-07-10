// combined.h -- combined SuperOS-606 + D650C-128 emulator build
// (SUPEROS_COMBINED). Scheme ported from SuperOS-303's combined build.
//
// One image, one firmware active per boot, selected by a byte in the internal
// EEPROM. SuperOS keeps its patterns in the flash-EEPROM arena (0xE000+); the
// d650c side keeps ALL of its state (the uPD444 pattern store) in the 4 KB
// internal EEPROM, which SuperOS never touches.
//
// Switching:
//   - SysEx `F0 7D 4D <fw> F7` from either firmware (fw: 0 = SuperOS, 1 = D650C)
//   - power-on escape: hold CLEAR + FUNCTION + PATTERN GROUP while switching
//     on -> boots SuperOS regardless of the stored selection (recovery path;
//     checked before either firmware starts, so the ROM never sees the keys)
//
// Internal EEPROM map (AT90USB1286, 4096 bytes):
//   0x000            firmware select: 0/0xFF = SuperOS, 1 = D650C
//   0x001            d650 area magic (EE_EMU_MAGIC_VAL when initialized)
//   0x008..0x01F     reserved (future d650 settings)
//   0x020..0x41F     d650 uPD444 packed pattern store (D650_EXT_BYTES = 1024)
//   0x420..0xFFF     free
#pragma once
#include <stdint.h>

#define EE_FW_SELECT   ((uint8_t *)0x000)
#define EE_EMU_MAGIC   ((uint8_t *)0x001)
#define EE_EMU_PATT    ((uint8_t *)0x020)

static constexpr uint8_t FW_SUPEROS = 0;   // also 0xFF (virgin EEPROM)
static constexpr uint8_t FW_D650    = 1;
static constexpr uint8_t EE_EMU_MAGIC_VAL = 0x66;

// Select the other firmware and reboot through the bootloader's app entry.
void combined_switch_firmware(uint8_t fw);
