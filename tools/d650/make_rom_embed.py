#!/usr/bin/env python3
"""make_rom_embed.py -- generate src/d650/rom_embed.h from a mask-ROM dump.

For PERSONAL builds only (env:combined-rom). The D650C mask ROM is
copyrighted: the generated header is gitignored and must never be committed
or distributed. The public build (env:combined) ships without a ROM and the
user uploads their own over MIDI (rom_store.h).

Input: either a raw 2048-byte .bin dump, or a recpu-format .syx transfer
(the same wire format rom_store.h receives):

  F0 7D 03 03 7E 03 <blk 0|1> <2048 nibbles hi,lo> <ck hi> <ck lo> F7
  F0 7D 03 03 7F 00 F7                                     (end marker)

Each block carries 1024 ROM bytes; <ck> is the mod-256 sum of the decoded
bytes, sent as two nibbles. Checksums are verified.

Usage: python3 tools/d650/make_rom_embed.py <rom.syx|rom.bin>
Writes: src/d650/rom_embed.h
"""
import sys
from pathlib import Path

ROM_SIZE = 2048
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "src" / "d650" / "rom_embed.h"


def decode_syx(data: bytes) -> bytes:
    rom = bytearray(ROM_SIZE)
    got = 0
    i = 0
    while i < len(data):
        if data[i] != 0xF0:
            i += 1
            continue
        end = data.find(0xF7, i)
        if end < 0:
            break
        msg = data[i + 1 : end]
        i = end + 1
        if len(msg) < 5 or msg[:4] != bytes((0x7D, 0x03, 0x03, 0x7E)):
            continue
        if msg[4] != 0x03 or len(msg) < 6:
            continue
        blk = msg[5]
        nibs = msg[6:]
        if blk > 1 or len(nibs) != 2 * 1024 + 2:
            sys.exit(f"error: malformed block message (blk={blk}, {len(nibs)} data bytes)")
        s = 0
        for k in range(1024):
            hi, lo = nibs[2 * k], nibs[2 * k + 1]
            if hi > 0x0F or lo > 0x0F:
                sys.exit("error: non-nibble data byte in block")
            by = (hi << 4) | lo
            rom[blk * 1024 + k] = by
            s = (s + by) & 0xFF
        ck = (nibs[2048] << 4) | nibs[2049]
        if ck != s:
            sys.exit(f"error: block {blk} checksum mismatch (file {ck:#04x}, computed {s:#04x})")
        got |= 1 << blk
        print(f"block {blk}: 1024 bytes, checksum {s:#04x} OK")
    if got != 0x03:
        sys.exit(f"error: incomplete transfer (blocks mask {got:#04x}, need 0x03)")
    return bytes(rom)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    src = Path(sys.argv[1])
    data = src.read_bytes()
    if len(data) == ROM_SIZE and 0xF0 not in data[:16]:
        rom = data
        print(f"raw binary dump: {ROM_SIZE} bytes")
    else:
        rom = decode_syx(data)
    sum16 = sum(rom) & 0xFFFF

    lines = [
        "// rom_embed.h -- GENERATED, PERSONAL BUILDS ONLY. DO NOT COMMIT.",
        "// The D650C mask ROM is copyrighted; this file is gitignored.",
        f"// Source: {src.name}, sum16 = 0x{sum16:04X}.",
        "// Regenerate: python3 tools/d650/make_rom_embed.py <rom.syx|rom.bin>",
        "#pragma once",
        "#include <avr/pgmspace.h>",
        "",
        f"static const uint8_t D650_ROM_EMBED[{ROM_SIZE}] PROGMEM = {{",
    ]
    for off in range(0, ROM_SIZE, 16):
        row = ", ".join(f"0x{b:02X}" for b in rom[off : off + 16])
        lines.append(f"  {row},")
    lines += ["};", ""]
    OUT.write_text("\n".join(lines))
    print(f"wrote {OUT} ({ROM_SIZE} bytes, sum16 0x{sum16:04X})")


if __name__ == "__main__":
    main()
