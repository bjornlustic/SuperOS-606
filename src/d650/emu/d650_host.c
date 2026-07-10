// d650_host.c — TR-606 machine glue. See d650_host.h.

// Hot I/O decode path; see the ucom4.c pragma note.
#if defined(SUPEROS_COMBINED) && defined(__AVR__)
#pragma GCC optimize("O3")
#endif

#include "d650_host.h"

#ifdef D650_HOST_TRACE_READS
// test-only observers: called on every PORTC/PORTA/PORTB read with latches intact
extern void d650_trace_read_c(d650_host *h);
extern void d650_trace_read_ab(d650_host *h, char port, uint8_t v);
#endif

// ---- external uPD444 RAM (2x 1024x4, IC7/IC8) --------------------------------
// Decode derived from the ROM's own access idioms (read prim @0x004, write
// prim @0x00C, step fetch @0x361) + schematic: PI1 = CE strobe (active high
// at the CPU). PE2 picks the chip (0 = IC7, 1 = IC8); per-chip address =
// PE3<<9 | PE0<<8 | PF<<4 | PD  (PD = rhythm 0-15, PF = step 0-15, PE0 =
// pattern group I/II — which is exactly why PE0 doubles as the I/II panel
// indicator; PE3 = which nibble bank). PE1 is the OH data line, NOT address.
// Step data: bank {PE2=0,PE3=1} -> PORTD nibble, {0,0} -> PORTF (LT/SD/BD/AC),
// {1,0} -> PORTE (CH/OH/CY/HT); {1,1} = counters/track scratch.
static inline uint8_t ext_on(const d650_host *h) {
  return ((h->lat_i >> 1) & 1) == D650_CE_ACTIVE;             // PI1 = CE gate
}
static inline uint16_t ext_addr(const d650_host *h) {
  return (uint16_t)(((h->lat_e >> 2) & 1) << 10)              // PE2 = chip
       | (uint16_t)(((h->lat_e >> 3) & 1) << 9)               // PE3 = A9
       | (uint16_t)((h->lat_e & 1) << 8)                     // PE0 = A8 (group I/II)
       | (uint16_t)(h->lat_f << 4)                            // PF  = step
       |  h->lat_d;                                           // PD  = rhythm
}
static inline uint8_t ext_get(const d650_host *h, uint16_t a) {
  return (h->ext[a >> 1] >> ((a & 1) << 2)) & 0x0F;
}
static inline void ext_put(d650_host *h, uint16_t a, uint8_t v) {
  uint8_t sh = (uint8_t)((a & 1) << 2);
  uint8_t old = h->ext[a >> 1];
  uint8_t nw  = (uint8_t)((old & (0xF0 >> sh)) | ((v & 0x0F) << sh));
  if (nw != old) { h->ext[a >> 1] = nw; h->ext_dirty = 1; }
}
// Write-transparent while WE is active and a chip is enabled: any
// data/address/strobe latch change commits the PORTC latch, like the real
// SRAM. The ROM only strobes with a stable address+data.
static void ext_refresh(d650_host *h) {
  if (((h->lat_i >> 0) & 1) != D650_WE_ACTIVE) return;
  if (!ext_on(h)) return;
  ext_put(h, ext_addr(h), h->lat_c);
}

// Selected scan row from PORTH (active-low, one row at a time; the ROM raises
// all four bits then clears one). All-high = the RUN/TAP/CLOCK status group.
static inline uint8_t row_of(uint8_t ph) {
  static const uint8_t lut[16] =
    { 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4 };
  return lut[ph & 0x0f];
}

// ---- ucom4 I/O callbacks (host is passed as `user`) -------------------------
static uint8_t io_read_a(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) { for (uint8_t j=0;j<4;j++) v |= (h->in[16 + row*4 + j] & 1) << j; }
  else {                                     // status group: RUN,TAP,-,CLOCK
    for (uint8_t j=0;j<4;j++) v |= (h->in[32 + j] & 1) << j;
    if (h->drv.read_clock)                   // live clock beats the snapshot
      v = (uint8_t)((v & 0x07) | ((h->drv.read_clock(h->drv.u) & 1) << 3));
  }
#ifdef D650_HOST_TRACE_READS
  d650_trace_read_ab(h, 'A', v & 0x0F);
#endif
  return v & 0x0F;
}
static uint8_t io_read_b(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) for (uint8_t j=0;j<4;j++) v |= (h->in[row*4 + j] & 1) << j;
#ifdef D650_HOST_TRACE_READS
  d650_trace_read_ab(h, 'B', v & 0x0F);
#endif
  return v & 0x0F;
}
static uint8_t io_read_c(void *u) {            // uPD444 data bus read
  d650_host *h = (d650_host *)u;
#ifdef D650_HOST_TRACE_READS
  d650_trace_read_c(h);
#endif
  if (!ext_on(h) || ((h->lat_i & 1) == D650_WE_ACTIVE)) return 0;
  return ext_get(h, ext_addr(h));
}
static uint8_t io_read_d(void *u) { (void)u; return 0; }

static void io_write(void *u, int port, uint8_t data) {
  d650_host *h = (d650_host *)u;
  data &= 0x0F;
  switch (port) {
    case UCOM4_PORTC: h->lat_c = data; ext_refresh(h); break; // uPD444 data latch
    case UCOM4_PORTD: h->lat_d = data; ext_refresh(h); break; // RAM A0-3 (rhythm #)
    case UCOM4_PORTE: h->lat_e = data; ext_refresh(h); break; // A8-9 + chip sel / CH,OH,CY,HT + I/II
    case UCOM4_PORTF: h->lat_f = data; ext_refresh(h); break; // RAM A4-7 (step #) / LT,SD,BD,AC
    case UCOM4_PORTG: h->lat_g = data; break;                 // STEP LED drive
    case UCOM4_PORTH: h->lat_h = data; break;                 // scan-select
    case UCOM4_PORTI: h->lat_i = data; ext_refresh(h); break; // WE, CE, COMMON TRIG
    default: return;
  }
  if (h->drv.port_write) h->drv.port_write(h->drv.u, port, data);
}

// ---- public API -------------------------------------------------------------
void d650_init(d650_host *h, const uint8_t *rom, const d650_drivers *drv) {
  for (unsigned i = 0; i < sizeof *h; i++) ((uint8_t*)h)[i] = 0;
  if (drv) h->drv = *drv;
  ucom4_io io = { io_read_a, io_read_b, io_read_c, io_read_d, io_write, h };
  ucom4_reset(&h->cpu, rom, &io);
}
uint32_t d650_step(d650_host *h) { return ucom4_step(&h->cpu); }
void d650_clock(d650_host *h, int level) { ucom4_set_int(&h->cpu, level); }
