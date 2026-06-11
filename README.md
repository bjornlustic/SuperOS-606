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
- **Flash persistence** — patterns survive power-off once `service-install.syx` is flashed

A companion web pattern editor (`tools/web-pattern-edit/`) sends/receives patterns over
MIDI SysEx.

## Build

Requires [PlatformIO](https://platformio.org/) and a git checkout (the SysEx packer
reads the git rev into the output filename).

```sh
pio run                  # app + bootloader, packs SuperOS-606_v*_<rev>.syx + .hex
pio run -e app           # just the app
pio run -e app-debug     # app with USB serial + DEBUG for bench testing
pio run -e flash-service # SPM flash-write service -> service-install.syx (install once)
```

Outputs (git-ignored): `SuperOS-606_v*.hex` (merged app + bootloader, for Teensy
Loader / ISP) and `SuperOS-606_v*.syx` (for the MIDI SysEx bootloader).

## Flashing

Two paths, depending on what the board exposes:

1. **Teensy Loader / ISP over USB** — flash the merged `SuperOS-606_v*.hex`.
2. **On-board MIDI SysEx bootloader** — hold the bootloader-entry button at power-on,
   then send `SuperOS-606_v*.syx` over MIDI IN (throttle the send).

### One-time setup — install the flash service

> **Do this once per board or your patterns will not survive power-off.**
> Pattern/track persistence (including patterns pushed from the web editor) goes
> through a tiny SPM flash-write service that must be installed one time:
>
> 1. Build it: `pio run -e flash-service` (produces `service-install.syx`).
> 2. Enter the bootloader (hold the bootloader-entry button at power-on).
> 3. Send `service-install.syx` over MIDI IN.
>
> Without it everything still runs — patterns just live in RAM and are lost when
> the unit powers off. Firmware updates never overwrite the installed service.

## Web pattern editor

`tools/web-pattern-edit/index.html` is a single-file, dependency-free web editor for
the 606's patterns and tracks: an 8×16 drum grid (the 7 voices + the global ACCENT
row), both pattern groups, per-pattern length (1-16) and scale, track chains (8 × 64),
a pattern library, undo/redo, and JSON save/load. Patterns live in the browser's
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
(or after a short idle) — **only if the one-time `service-install.syx` setup
above has been done**.

Web MIDI needs Chrome/Edge and an HTTP origin (not `file://`):

```sh
python3 -m http.server 8080 --directory tools/web-pattern-edit
```

or just press F5 in VS Code / Cursor — `.vscode/launch.json` serves the editor on
port 3000 with live reload and opens it in Chrome.

## License

MIT — see [LICENSE](LICENSE). Copyright © 2026 Bjorn Lustic.
