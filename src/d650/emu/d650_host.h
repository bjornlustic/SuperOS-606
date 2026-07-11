// d650_host.h — the emulated TR-606 "machine": uPD650C-128 core + external
// uPD444 pattern RAM + I/O decode onto TR-606 semantics. Portable C (no
// AVR/Arduino deps) so it runs identically on desktop (validation) and on the
// AT90USB1286. Structure follows the SuperOS-303 d650_host (same CPU socket,
// different port meanings — service manual p.2 "µPD650C functional description").
//
// The host implements the four ucom4 I/O callbacks:
//   read A/B  -> switch/status matrix (from an input snapshot, see below)
//   read C    -> uPD444 data bus (only while a chip is enabled and WE idle)
//   write C..I-> uPD444 RAM bus; everything else is forwarded raw to the
//                platform via drv.port_write (LED matrix PG/PH, instrument
//                data PF/PE, COMMON TRIG PI2) — on hardware those latches ARE
//                the physical pin levels, so the AVR bridge just mirrors them.
//
// External RAM model (service manual p.2/p.4):
//   2x uPD444C (IC7/IC8), 1024x4 each = 2048 nibbles. Data on PC via 3.3k
//   (R76-79). Address: PD = A0..A3 ("rhythm number"), PF = A4..A7 ("step
//   number"), PE0/PE1 = A8/A9. Chip select: PE2 -> IC7, PE3 -> IC8, gated by
//   PI1 ("Memory CE") through IC6 (HD14011 NAND); PI0 = "Memory WE", shared,
//   33k pullup (R83). CPU-side strobe polarity is behind the D650_*_ACTIVE
//   macros below — confirmed by observing the emulated machine in the host
//   testbench, not guessed from the gate drawing.
#pragma once
#include <stdint.h>
#include "ucom4.h"

#ifdef __cplusplus
extern "C" {
#endif

// Input snapshot, same layout as SuperOS-606 hw::ScanMatrix produces
// (controls.h): 1 = closed/active cell.
//   [row*4 + col]      PB while PH row selected: 16 STEP switches   idx  0..15
//   [16 + row*4 + col] PA while PH row selected: MODE/SCALE/INST/
//                      FUNCTION/CLEAR/GROUP                         idx 16..31
//   [32 + j]           PA with all PH high (status): RUN, TAP, -, TEMPO CLOCK
#define D650_IN_COUNT 40
#define D650_IN_RUN   32
#define D650_IN_TAP   33
#define D650_IN_CLOCK 35

// External uPD444 store: 2048 nibbles packed 2 per byte (even nibble = low 4
// bits). The packed array is the persistence image — snapshot/restore verbatim.
#define D650_EXT_NIBS  2048
#define D650_EXT_BYTES (D650_EXT_NIBS / 2)

// Strobe polarity at the CPU pins, confirmed by observing the emulated
// machine's uPD444 accesses in the host testbench (not guessed from the gate
// drawing).
#ifndef D650_WE_ACTIVE
#define D650_WE_ACTIVE 1   // PI0 pulses HIGH around a write
#endif
#ifndef D650_CE_ACTIVE
#define D650_CE_ACTIVE 1   // PI1 pulses HIGH around any access
#endif

// Driver hook — set by the platform. The host forwards every port write
// (C..I, ucom4 port index + 4-bit latch value) after its own RAM decode; the
// AVR bridge mirrors PG/PH to the LED matrix, PF/PE to the instrument-data
// pins and PI bit2 to COMMON TRIG. Desktop mocks log/inspect.
//
// read_clock (optional, may be NULL): live TEMPO/DIN clock level, sampled at
// the instant the emulated ROM reads its status group (PA with all PH high).
// The 4 ms snapshot poll is too coarse for the 1.8 ms-cadence sampling the
// real D650C did; on the AVR the PH latches are mirrored to the real select
// pins, so at that exact moment the physical clock line is readable directly
// (it is actively driven, so it needs none of the key lines' settle time).
// NULL = fall back to the in[D650_IN_CLOCK] snapshot (desktop testbench).
typedef struct {
  void (*port_write)(void *u, int port, uint8_t data);
  void *u;
  uint8_t (*read_clock)(void *u);
} d650_drivers;

typedef struct {
  ucom4_t  cpu;
  uint8_t  ext[D650_EXT_BYTES]; // uPD444 x2 pattern/track store, packed nibbles
  uint8_t  ext_dirty;    // set when a pattern-RAM nibble changes (save trigger)
  uint8_t  in[D650_IN_COUNT];
  // Output-latch mirrors (the core updates its port_out AFTER the write
  // callback runs, so the host keeps its own copies).
  uint8_t  lat_c, lat_d, lat_e, lat_f, lat_g, lat_h, lat_i;
  d650_drivers drv;
} d650_host;

// Initialize + reset. `rom` = 2 KB TR-606 mask ROM. `drv` may be NULL.
void d650_init(d650_host *h, const uint8_t *rom, const d650_drivers *drv);

// Run one CPU instruction. Returns machine cycles consumed.
uint32_t d650_step(d650_host *h);

// Run at least `cycles` machine cycles (batched; cheaper than stepping).
static inline uint32_t d650_run(d650_host *h, uint32_t cycles) {
  return ucom4_run(&h->cpu, cycles);
}

// Drive the /INT pin (the fixed ~556 Hz "interrupt clock", schematic p.4:
// 1.8 ms period — NOT the tempo clock; tempo goes on in[D650_IN_CLOCK]).
// Rising edge latches the interrupt.
void d650_clock(d650_host *h, int level);

// Snapshot/restore the packed pattern store (persistence layer target).
static inline int d650_dirty(const d650_host *h) { return h->ext_dirty; }
static inline void d650_clear_dirty(d650_host *h) { h->ext_dirty = 0; }

#ifdef __cplusplus
}
#endif
