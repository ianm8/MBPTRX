// asyncI2S.h
//
// 24-bit I2S receiver for the PCM1808 (slave mode) on the RP2350 PIO.
//
// The PIO is the I2S master: it drives BCK and LRCK by side-set and samples
// DOUT. MCLK (PCM1808 SCKI, 256 x fs) comes from a PWM slice. All three are
// integer divisions of the system clock, so they stay locked together, as the
// PCM1808 needs in slave mode. At 240 MHz:
//
//   fs = 31250: MCLK =  8 MHz (sys/30), BCK = 2 MHz, PIO clock = sys/30
//   fs = 62500: MCLK = 16 MHz (sys/15), BCK = 4 MHz, PIO clock = sys/15
//
// Each RX FIFO word is one channel, in the same alignment as the I2S library
// read32():
//
//   bits 31..8  24-bit sample, two's complement
//   bits  7..1  zero
//   bit   0     channel tag: 0 = left (I), 1 = right (Q)
//
// The tag takes the place of the I2S delay bit (always zero from the PCM1808),
// so I and Q can never be swapped, even after words are dropped.
//
// The PIO never stalls. When the FIFO is full the word is dropped
// (push noblock), so BCK and LRCK keep running and the PCM1808 stays in sync
// while core 1 is parked (EEPROM save).
//
// Usage, as with the uDST data ready strobe: in loop1() return until level()
// shows the frames needed for one output sample, then read() them.
// The RX FIFO is joined: 8 words = 4 frames.
//
// Call begin() from setup() or setup1(), not from a global constructor:
// arduino-pico sets the system clock after global constructors have run.
#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// PIO program. Side-set bit 0 = BCK, bit 1 = LRCK. 4 PIO cycles per bit,
// 32 bits per channel, 64 bits per frame. Y = 29.
//
// LRCK changes with BCK falling, and the first BCK rising edge after it still
// belongs to the previous channel (I2S one bit delay). That bit is replaced
// by the channel tag and the word is pushed.
//
//    0: nop             side 0b00 [1]  ; BCK low, LRCK low (left slot)
//    1: in     y, 1     side 0b01      ; BCK high: tag 1, last bit of right word
//    2: push   noblock  side 0b01      ; right word done
//    3: mov    x, y     side 0b00 [1]
//    4: in     pins, 1  side 0b01 [1]  ; left bits 1..30
//    5: jmp    x--, 4   side 0b00 [1]
//    6: in     pins, 1  side 0b01 [1]  ; left bit 31
//    7: nop             side 0b10 [1]  ; BCK low, LRCK high (right slot)
//    8: in     null, 1  side 0b11      ; tag 0, last bit of left word
//    9: push   noblock  side 0b11      ; left word done
//   10: mov    x, y     side 0b10 [1]
//   11: in     pins, 1  side 0b11 [1]  ; right bits 1..30
//   12: jmp    x--, 11  side 0b10 [1]
//   13: in     pins, 1  side 0b11 [1]  ; right bit 31
// ---------------------------------------------------------------------------
#define i2s_rx_24_wrap_target 0
#define i2s_rx_24_wrap        13

static const uint16_t i2s_rx_24_program_instructions[] =
{
          //     .wrap_target
  0xa142, //  0: nop                    side 0 [1]
  0x4841, //  1: in     y, 1            side 1
  0x8800, //  2: push   noblock         side 1
  0xa122, //  3: mov    x, y            side 0 [1]
  0x4901, //  4: in     pins, 1         side 1 [1]
  0x0144, //  5: jmp    x--, 4          side 0 [1]
  0x4901, //  6: in     pins, 1         side 1 [1]
  0xb142, //  7: nop                    side 2 [1]
  0x5861, //  8: in     null, 1         side 3
  0x9800, //  9: push   noblock         side 3
  0xb122, // 10: mov    x, y            side 2 [1]
  0x5901, // 11: in     pins, 1         side 3 [1]
  0x114b, // 12: jmp    x--, 11         side 2 [1]
  0x5901, // 13: in     pins, 1         side 3 [1]
          //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program i2s_rx_24_program =
{
  .instructions = i2s_rx_24_program_instructions,
  .length = 14,
  .origin = -1,
  .pio_version = 0,
#if PICO_PIO_VERSION > 0
  .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config i2s_rx_24_program_get_default_config(uint offset)
{
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, offset + i2s_rx_24_wrap_target, offset + i2s_rx_24_wrap);
  sm_config_set_sideset(&c, 2, false, false);
  return c;
}
#endif

class AsyncI2S
{
  public:
    AsyncI2S(PIO pio, uint sm, uint dout_pin, uint bck_pin, uint mclk_pin, uint32_t fs);
    bool begin(void);
    uint32_t level(void);
    bool read(int32_t &i, int32_t &q);
    uint32_t resyncs;

  private:
    PIO      pio_;
    uint     sm_;
    uint     dout_pin_;
    uint     bck_pin_;
    uint     mclk_pin_;
    uint32_t fs_;
};

inline AsyncI2S::AsyncI2S(PIO pio, uint sm, uint dout_pin, uint bck_pin, uint mclk_pin, uint32_t fs)
    : resyncs(0), pio_(pio), sm_(sm), dout_pin_(dout_pin), bck_pin_(bck_pin),
      mclk_pin_(mclk_pin), fs_(fs)
{
}

// Set up MCLK and the PIO. LRCK is on bck_pin + 1.
// Returns false if the system clock is not an exact multiple of the
// clocks needed (the PCM1808 slave mode needs exact ratios).
inline bool AsyncI2S::begin(void)
{
  const uint32_t sys = clock_get_hz(clk_sys);

  // PIO: 4 cycles per bit, 64 bits per frame
  const uint32_t pio_hz = fs_ * 64u * 4u;
  const uint32_t pio_div = sys / pio_hz;
  if (pio_div == 0u || pio_div * pio_hz != sys)
  {
    return false;
  }

  // MCLK = 256 x fs
  const uint32_t mclk_hz = fs_ * 256u;
  const uint32_t mclk_div = sys / mclk_hz;
  if (mclk_div < 2u || mclk_div * mclk_hz != sys)
  {
    return false;
  }

  // MCLK from PWM: period mclk_div system clocks, about 50% duty
  gpio_set_function(mclk_pin_, GPIO_FUNC_PWM);
  const uint slice = pwm_gpio_to_slice_num(mclk_pin_);
  const uint chan = pwm_gpio_to_channel(mclk_pin_);
  pwm_set_clkdiv_int_frac(slice, 1, 0);
  pwm_set_wrap(slice, (uint16_t)(mclk_div - 1u));
  pwm_set_chan_level(slice, chan, (uint16_t)(mclk_div / 2u));
  pwm_set_enabled(slice, true);

  // load the program and build the config
  const uint offset = pio_add_program(pio_, &i2s_rx_24_program);
  pio_sm_config c = i2s_rx_24_program_get_default_config(offset);

  // hand the pins to the PIO: DOUT in, BCK and LRCK out
  pio_gpio_init(pio_, dout_pin_);
  pio_gpio_init(pio_, bck_pin_);
  pio_gpio_init(pio_, bck_pin_ + 1u);
  sm_config_set_in_pins(&c, dout_pin_);
  sm_config_set_sideset_pins(&c, bck_pin_);

  // MSB first: shift left, explicit push (no autopush)
  sm_config_set_in_shift(&c, /*shift_right=*/false, /*autopush=*/false, /*push_thresh=*/32);

  // 8 word RX FIFO
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  sm_config_set_clkdiv_int_frac(&c, (uint16_t)pio_div, 0);

  pio_sm_set_consecutive_pindirs(pio_, sm_, dout_pin_, 1, false);
  pio_sm_set_consecutive_pindirs(pio_, sm_, bck_pin_, 2, true);
  pio_sm_set_pins_with_mask(pio_, sm_, 0u, 3u << bck_pin_);

  pio_sm_init(pio_, sm_, offset, &c);

  // Y = 29: bits 1..31 of each channel (30 in the loop, 1 after it)
  pio_sm_exec(pio_, sm_, pio_encode_set(pio_y, 29));

  pio_sm_clear_fifos(pio_, sm_);
  pio_sm_set_enabled(pio_, sm_, true);
  return true;
}

// words waiting in the RX FIFO (2 per frame)
inline uint32_t AsyncI2S::level(void)
{
  return pio_sm_get_rx_fifo_level(pio_, sm_);
}

// Read one frame (I = left, Q = right). Returns false, and drops words,
// if fewer than 2 words are waiting or the channel tags are out of order.
inline bool AsyncI2S::read(int32_t &i, int32_t &q)
{
  if (pio_sm_get_rx_fifo_level(pio_, sm_) < 2u)
  {
    return false;
  }
  const uint32_t w0 = pio_sm_get(pio_, sm_);
  if ((w0 & 1u) != 0u)
  {
    // right word where a left word should be
    resyncs++;
    return false;
  }
  const uint32_t w1 = pio_sm_get(pio_, sm_);
  if ((w1 & 1u) == 0u)
  {
    // left word where a right word should be
    resyncs++;
    return false;
  }
  i = (int32_t)(w0 & 0xffffff00u);
  q = (int32_t)(w1 & 0xffffff00u);
  return true;
}