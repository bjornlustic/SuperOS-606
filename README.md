# SuperOS-606

Open firmware for the **Roland TR-606 "Drumatix"**. It runs on a Teensy++ 2.0
(AT90USB1286) fitted as a drop-in replacement for the TR-606's original NEC µPD650
CPU, re-implementing the drum machine's sequencer in modern, readable C++.

## Status

🟡 **Sequencer bring-up, awaiting hardware verification.** Implemented:

- **Pattern write** — step entry, tap write, chase-delete, length, scale, global accent
- **Pattern play** — selection, range chains, pattern groups I/II
- **Track play / write** — the INSTRUMENT dial selects track 1-8
- **Transport + 24 PPQN timing** read from the 606's own START/STOP flip-flop and TEMPO clock
- **MIDI out** — clock / start / stop, plus a note per drum hit
- **Flash persistence** — patterns survive power-off; the SPM flash service ships
  inside both release images (no separate install step)

A companion web pattern editor (`tools/web-pattern-edit/`) sends/receives patterns over
MIDI SysEx.

## Build

Requires [PlatformIO](https://platformio.org/) and a git checkout (the SysEx packer
reads the git rev into the output filename).

```sh
pio run -e bootloader -e flash-service -e combined
                         # bootloader + service + combined app; packs the app
                         # .syx and merges SuperOS-606_v*_combined_<rev>-full.hex
                         # (the merge happens on the app build, so build the
                         # bootloader and service first / at the same time)
pio run -e app           # just the plain 4 MHz app
pio run -e app-debug     # app with USB serial + DEBUG for bench testing
```

Outputs (git-ignored): `SuperOS-606_v*-full.hex` (app + bootloader + SPM flash
service, for ISP) and `SuperOS-606_v*_<env>_<rev>.syx` (app + flash service, for
the MIDI SysEx bootloader). Both images carry the flash service, so one install
is complete on its own; `service-install.syx` still exists for installing just
the service on a board that has the bootloader but lost the service.

## Flashing

Two paths:

1. **ISP (Atmel-ICE / usbasp)** — flash the merged `SuperOS-606_v*-full.hex`
   (app + MIDI bootloader + SPM flash service in one image):

   ```sh
   pio run -e bootloader -e flash-service -e combined   # builds + merges the full hex
   avrdude -c atmelice_isp -p usb1286 -U flash:w:SuperOS-606_v*_combined_*-full.hex:i
   ```

   **Always use the `-full` hex over ISP.** `avrdude -U flash:w:` chip-erases the
   whole part first, so ISP-flashing a bare app hex silently deletes the MIDI
   bootloader and the flash service: TAP-at-power-on stops entering the
   bootloader (the unit just boots the app) and patterns stop surviving
   power-off. The chip erase also wipes the internal EEPROM (D650C patterns +
   the uploaded mask ROM) unless the EESAVE fuse is programmed (hfuse `0xD2`
   instead of `0xDA`).

2. **On-board MIDI SysEx bootloader** (no ISP tool needed, once the full image
   is on the board): hold **WRITE/NEXT (TAP)** while powering on. Step LEDs 1-4
   blink twice, then step 1 stays lit solid: the bootloader is waiting. Send
   the app `.syx` with the throttled sender:

   ```sh
   python3 tools/send_syx.py SuperOS-606_v*_combined_<rev>.syx -p "Your MIDI Port"
   ```

   Step LEDs 1-4 cycle while pages write; the unit boots the new app when the
   final EXECUTE message lands. If LEDs 1-4 blink together forever, a page
   checksum failed: power-cycle into the bootloader and send again with a
   bigger delay (`-d 0.2`). SysEx flashing rewrites the app pages and refreshes
   the flash service, never the bootloader or the EEPROM.

## Web pattern editor

`tools/web-pattern-edit/index.html` is a single-file, dependency-free web editor for
the 606's patterns and tracks: a drum grid (the 7 voices + the global ACCENT row)
that grows to the pattern length (up to 64 steps), both pattern groups, per-pattern
length and scale, per-voice loop length with bar-reset/polymeter, per-step ratchets,
track chains (8 × 64), a pattern library, undo/redo, and JSON save/load. Patterns live in the browser's
local storage, so it also works fully offline as a scratchpad.

With the 606 connected over MIDI the editor and the hardware stay in sync
(protocol in `src/midi_api.h`):

- **On connect** (automatic once the browser has MIDI permission — just open
  the page) it reads all 32 patterns + 8 tracks from the 606 behind a loading
  bar. Panel writes mirror back live: step entry, tap write, chase-delete,
  length, scale, and PATTERN CLEAR on the 606 all appear in the grid as they
  happen.
- **Every grid edit mirrors to the 606 as you make it** — a painted step
  sounds when the sequencer reaches it. MIDI Note Ons matching the drum map
  one-shot-trigger voices and Program Change 0-31 selects patterns, so pads,
  controllers, and DAWs work too.
- **Clicking a pattern pill selects it on the 606** (immediately when stopped,
  at the next pattern wrap while running), and **dragging across pills chains
  up to 4 patterns**, like the 303 editor. The 606's LEDs and group indicator
  follow. A clock-derived playhead chases the grid.
- **Edit Live** detaches the editor so you can edit any pattern while the 606
  keeps playing another — like the 303 editor's Edit Live. Edits still mirror
  to the pattern's slot on the 606 either way.

Pushed patterns land in RAM immediately and persist to flash at the next STOP
(or after a short idle); the flash service that makes this work is part of the
release images.

Web MIDI needs Chrome/Edge and an HTTP origin (not `file://`):

```sh
python3 -m http.server 8080 --directory tools/web-pattern-edit
```

or just press F5 in VS Code / Cursor — `.vscode/launch.json` serves the editor on
port 3000 with live reload and opens it in Chrome.

## D650C emulator mode and the mask ROM

The combined build (`pio run -e combined`) can boot either SuperOS-606 or a
cycle-accurate emulator of the original NEC D650C-128 CPU, selected per boot
(SuperOS config menu: hold FUNCTION, tap CLEAR, then press step 9; or SysEx
`F0 7D 4D 01 F7`).

The original mask ROM is copyrighted and is **not** included in this firmware
or repository. To use the emulator you must supply your own dump of the D650C
ROM, e.g. dumped from your own unit, as permitted by your local law. The flow:

1. Flash the combined firmware. It boots SuperOS; the emulator has no ROM yet.
2. Switch to D650C mode (config menu, step 9). With no ROM stored the 606
   enters **ROM upload mode**: step LEDs 1 and 9 blink alternately while it
   waits.
3. In the web editor, use **D650C mask ROM → Load ROM dump** and pick your
   dump: a raw 2048-byte `.bin`, or a `.syx` already in the RE-303/recpu
   nibble format. The editor checksums it and tells you whether it matches the
   known-good TR-606 ROM (sha1 `ae605ce2…`).
4. **Send to 606** (or save the converted `.syx` and send it with any SysEx
   tool). The transfer takes about 1.5 s, the 606 stores the ROM in its
   internal EEPROM (a few seconds) and reboots straight into the emulator.

The upload is checksummed per block and the stored copy is validated at every
boot, so a failed or interrupted transfer just leaves the 606 waiting in
upload mode. Sending a ROM to a **running** emulator also works (it freezes,
stores, and reboots), so a ROM can be replaced later. To leave upload mode
without a ROM, use the normal config menu: hold FUNCTION, tap CLEAR (step 9
blinks), then press step 9 to boot SuperOS. Alternatives: send
`F0 7D 4D 00 F7`, or power on holding CLEAR + FUNCTION + PATTERN GROUP.

## License

MIT — see [LICENSE](LICENSE). Copyright © 2026 Bjorn Lustic.
