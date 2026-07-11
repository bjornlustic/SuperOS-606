# Provenance

`emu/ucom4.{h,c}` are copied verbatim from the SuperOS-303 combined branch
(`SuperOS-303FIRMWARE` @ `combined650csuperOS`, itself synced from
`SuperOS-303D650cEmulator` commit `1636136`) — the MAME-derived uCOM-43 core,
lockstep-verified against MAME's own `ucom4op.cpp` over 100 M instructions.

`emu/d650_host.{h,c}` is the TR-606 machine layer, written for this repo:
same structure as the 303 host, but with the 606's port decode taken from the
service manual p.2/p.4 and validated against the emulator's host testbench.

The D650C mask ROM is copyrighted and is NOT shipped with the firmware. The
user uploads their own dump over MIDI SysEx (RE-303/recpu nibble format, see
`rom_store.h`); it is stored in the internal EEPROM and executed from an SRAM
copy.

`emu_avr.cpp` is the 606 AVR bridge (pacing constants and batching policy
ported from the 303 bridge, I/O layer written for the 606 socket wiring).
