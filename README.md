# SuperOS-606

Open-source super firmware for the **Roland TR-606 "Drumatix"**, running on a drop-in replacement
for the original NEC µPD650 CPU (Teensy++ 2.0 / AT90USB1286).

The replacement board re-implements the µPD650 interface in firmware, so the rest of
the TR-606 — switch/LED matrix, voice trigger circuits, analog drum board — runs
unmodified. The µPD650 port map is documented inline in [src/pins.h](src/pins.h).

## Status

The current power-on firmware is a **voice button test**: step buttons 1-7 are mapped to the seven drum voices (BD, SD, LT, HT, CH, OH, CY) so each instrument can be played by hand to confirm it
triggers.

## Build

Requires [PlatformIO](https://platformio.org/) and a git repo (the syx packer reads the
git rev).

```sh
pio run                 # builds app + bootloader, packs SuperOS-606_vX_<rev>.syx + .hex
pio run -e app          # just the app
pio run -e app-debug    # app with USB serial + DEBUG for bench testing
```

Outputs: `SuperOS-606_v*.hex` (merged app+bootloader, for Teensy Loader/ISP)
and `SuperOS-606_v*.syx` (for the MIDI SysEx bootloader).

## Flashing

Flash via MIDI Sysex.

## License

Released under the [MIT License](LICENSE) — © 2026 Bjorn Lustic.

## Credits

Bjorn Lustic 2026
