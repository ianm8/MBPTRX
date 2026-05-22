/*
 * MBPTRX Version 5.0.240
 *
 * Copyright 2026 Ian Mitchell VK7IAN
 * Licenced under the GNU GPL Version 3
 *   (unless otherwise indicated)
 *
 * Libraries
 *
 *  https://github.com/Bodmer/TFT_eSPI
 *
 * Filter Design
 *
 *  https://www.arc.id.au/FilterDesign.html
 *
 * Important:
 *   Copy "User_Setup.h" to  ..\Arduino\libraries\TFT_eSPI whenever the TFT_eSPI library is installed
 *   Important Note, there is a bug in version that requires the following change in TFT_eSPI_RP2040.h:
 *
 *     #define SET_BUS_READ_MODE  // spi_set_format(SPI_X,  8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST)
 *
 *   that is, comment out: spi_set_format(SPI_X,  8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST)
 *
 * Build:
 *  Pi Pico 2
 *  CPU Speed: 240Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 4MB (no FS)
 *
 * Some history
 *  0.1.240 start with uSBITX and iqSDR40 code
 *  0.2.240 fix read I2C port bug
 *  0.3.240 fix RX/TX relay bug
 *  0.4.240 spectrum level
 *  0.5.240 direct CW
 *  0.6.240 complex mixer offset
 *  0.7.240 update bandwidths
 *  0.8.240 enable EEPROM writes
 *  0.9.240 fix CW display width
 *  1.0.240 RX bandwith options
 *  1.1.240 code cleanup
 *  1.2.240 fix SWR display
 *  1.3.240 graph power and SWR
 *  1.4.240 set spectrum level
 *  1.5.240 exit level if TX
 *  1.6.240 smeter sensitivity higher bands
 *  1.7.240 change TX frequency colour
 *  1.8.240 fix CW display position
 *  2.0.240 AM mode for SWL band
 *  2.1.240 improved AM demodulation
 *  2.2.240 include modified TFT_eSPI library
 *  2.3.240 fix TFT display initialisation
 *  2.4.240 separate modified TFT_eSPI library
 *  2.5.240 add exit option to JNR menu
 *  2.6.240 noise blanker
 *  2.7.240 TX/RX transition glitch
 *  2.8.240 reposition status info
 *  2.9.240 tweak noise blanker
 *  3.0.240 mic processor
 *  3.1.240 CW decoder - adaptive
 *  3.2.240 CW decoder - schmitt
 *  3.3.240 spectrum set or adjust
 *  3.4.240 update fonts
 *  4.0.240 digital mode with Vox
 *  4.1.240 don't show JNR in DIG mode
 *  4.2.240 lower sideband digital
 *  4.3.240 TX guard on startup
 *  4.4.240 minor update to spectrum processing
 *  5.0.240 FT8 built in
 */

//#define DEBUGGING_SKIP

#include <SPI.h>
#include <EEPROM.h>
#include <I2S.h>
#include <TFT_eSPI2.h>
#include "util.h"
#include "si5351.h"
#include "mcp3021.h"
#include "TCA9534A.h"
#include "Rotary.h"
#include "spectrum.h"
#include "dsp.h"
#include "menu.h"
#include "cw.h"
#include "cwdecode1.h"
#include "cwdecode2.h"
#include "ft8.h"
#include "ft8ui.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "ArialBold14pt7b.h"
#include "ArialBold16pt7b.h"

#define YOUR_CALL "VK7IAN"
#define YOUR_GRID "QE36"
#define POS_CALL_X 70

#if SI5351_PLL_VCO_MIN != 440000000
#err set SI5351_PLL_VCO_MIN to 440000000 in si5351.h
#endif

#define VERSION_STRING "  V5.0."
#define CW_TIMEOUT 800u
#define MENU_TIMEOUT 5000u
#define VOX_LEVEL 100u
#define VOX_TIMEOUT 250u
#define BAND_80M 0
#define BAND_40M 1
#define BAND_30M 2
#define BAND_20M 3
#define BAND_17M 4
#define BAND_15M 5
#define BAND_12M 6
#define BAND_10M 7
#define BAND_SWL 8
#define BAND_MIN BAND_80M
#define BAND_MAX BAND_SWL
#define NUM_BANDS 9
#define DEFAULT_FREQUENCY 7100000ul
#define DEFAULT_BAND BAND_40M
#define DEFAULT_MODE MODE_LSB
#define DEFAULT_STEP 1000ul
#define DEFAULT_CW_SPEED 60ul
#define DEFAULT_SIDETONE 700ul
#define DEFAULT_CW_LEVEL 2ul
#define DEFAULT_MICGAIN 100ul
#define DEFAULT_MICPROC 0ul
#define DEFAULT_BANDWIDTH 3ul
#define BUTTON_LONG_PRESS_TIME 800ul
#define TCXO_FREQ 27000021ul
#define SPECTRUM_LEVEL_MIN -4
#define SPECTRUM_LEVEL_MAX 4
#define LPF_I2C_ADDRESS 0x20U
#define BPF_I2C_ADDRESS 0x21U

// sets FT8 output power (max 511)
#define FT8_TX_MAX 127u 

#define PIN_PTT      0 // Mic PTT (active low)
#define PIN_ENCBUT   1 // on rotary
#define PIN_ENCA     2 // rotary
#define PIN_ENCB     3 // rotary
#define PIN_SDA      4 // I2C
#define PIN_SCL      5 // I2C
#define PIN_TX000    6 // TX PWM
#define PIN_TX180    7 // TX PWM
#define PIN_TX090    8 // TX PWM
#define PIN_TX270    9 // TX PWM
#define PIN_DOUT    10 // I2S
#define PIN_BCLK    11 // I2S
#define PIN_LRCL    12 // I2S
#define PIN_MCLK    13 // I2S
#define PIN_AUPWML  14 // audio PWM low
#define PIN_AUPWMH  15 // audio PWM high
#define PIN_MISO2   16 // LCD see User_Setup.h (not used)
#define PIN_CS      17 // LCD see User_Setup.h
#define PIN_SCK     18 // LCD see User_Setup.h
#define PIN_MOSI2   19 // LCD see User_Setup.h
#define PIN_DC      20 // LCD see User_Setup.h
#define PIN_RST     21 // LCD see User_Setup.h
#define PIN_TXBIAS  22 // enable TX bias
#define PIN_REG     23 // Pico regulator
#define PIN_PADA    26 // CW Paddle A
#define PIN_PADB    27 // CW Paddle B
#define PIN_MIC     28 // analog MIC

// these "Pins" used with TCA9534
#define I2C_PIN_LPF1     0 // low-pass filter
#define I2C_PIN_LPF2     1 // low-pass filter
#define I2C_PIN_LPF3     2 // low-pass filter
#define I2C_PIN_LPF4     3 // low-pass filter
#define I2C_PIN_TXENABLE 4 // low to engage TX relay
#define I2C_PIN_BPF1     0 // band-pass filer low bit
#define I2C_PIN_BPF2     1 // band-pass filer high bit
#define I2C_PIN_TXN      2 // low to enable QSE
#define I2C_PIN_RXN      3 // low to enable QSD
#define I2C_PIN_MUTE     4 // low to mute audio

// width and height of LCD
#define LCD_WIDTH         240
#define LCD_HEIGHT        135
#define POS_SPLASH_X       55
#define POS_SPLASH_Y       60
#define POS_VERSION_X       0
#define POS_VERSION_Y     122
#define POS_FREQUENCY_X    72
#define POS_FREQUENCY_Y    21
#define POS_TX_X            4
#define POS_TX_Y           17
#define POS_RX_X           35
#define POS_RX_Y           17
#define POS_MODE_X         10
#define POS_MODE_Y         30
#define POS_BAND_X         57
#define POS_BAND_Y         30
#define POS_CPU_X         199
#define POS_CPU_Y          30
#define POS_JNR_X         199
#define POS_JNR_Y          52
#define POS_DEBUG_X        20
#define POS_DEBUG_Y        50
#define POS_ATT_X         190
#define POS_ATT_Y          30
#define POS_MULTI_X        55
#define POS_MULTI_Y        30
#define POS_MULTIVALUE_X   80
#define POS_MULTIVALUE_Y   65
#define POS_METER_X       100
#define POS_METER_Y        25
#define POS_TUNING_STEP_X 130
#define POS_TUNING_STEP_Y  40
#define POS_SWR_X         125
#define POS_SWR_Y          40
#define POS_WATER_X         0
#define POS_WATER_Y        62
#define POS_CW_SETTINGS_X   0
#define POS_CW_SETTINGS_Y  52
#define POS_CESSB_MIC_X     0
#define POS_CESSB_MIC_Y    52
#define POS_CENTER_LEFT   119
#define POS_CENTER_RIGHT  120
#define POS_MENU_X         40
#define POS_MENU_Y         30
#define MENU_WIDTH        160
#define MENU_HEIGHT        82
#define SPECTRUM_WIND      0u
#define SPECTRUM_GRASS     1u
#define JNR_OFF            0u
#define JNR_LEVEL1         1u
#define JNR_LEVEL2         2u
#define JNR_LEVEL3         3u
#define NB_OFF             0u
#define NB_LEVEL1          1u
#define NB_LEVEL2          2u
#define NB_LEVEL3          3u
#define NB_LEVEL4          4u
#define NB_LEVEL5          5u
#define MIC_PROC1          1u
#define MIC_PROC2          2u
#define MIC_PROC3          3u
#define MIC_PROC4          4u
#define MIC_PROC5          5u
#define MIC_PROC_OFF       0u

#if PIN_MIC == 26U
#define MIC_MUX 0U
#elif PIN_MIC == 27U
#define MIC_MUX 1U
#elif PIN_MIC == 28U
#define MIC_MUX 2U
#elif PIN_MIC == 29U
#define MIC_MUX 3U
#endif

// two buffers of 1024 FFT bins
#define MAX_ADC_SAMPLES 2048
#define SPECTRUM_BUFFER 1024
#define WATERFALL_ROWS 41

static_assert((MAX_ADC_SAMPLES & (MAX_ADC_SAMPLES - 1)) == 0, "MAX_ADC_SAMPLES must be a power of 2");

#define ERROR_SYSCLOCK 3u
#define ERROR_SI5351   4u
#define ERROR_LPF      5u
#define ERROR_BPF      6u
#define ERROR_FWDADC   7u
#define ERROR_REFADC   8u

enum radio_mode_t
{
  MODE_LSB,
  MODE_USB,
  MODE_CWL,
  MODE_CWU,
  MODE_DGL,
  MODE_DGU,
  MODE_FT8,
  MODE_AM
};

volatile static struct
{
  int32_t tune;
  uint32_t step;
  uint32_t frequency;
  uint32_t band;
  uint32_t cw_dit;
  uint32_t cw_level;
  uint32_t sidetone;
  uint32_t cw_phase;
  uint32_t cwdecode;
  uint32_t spectype;
  uint32_t jnrlevel;
  uint32_t nblevel;
  uint32_t micgain;
  uint32_t micproc;
  uint32_t bandwidth;
  radio_mode_t mode;
  bool tx_safe;
  bool tx_button;
  bool tx_enable;
  bool keydown;
  bool cessb;
  bool gaussian;
  bool menu_active;
  bool mode_auto;
  bool graph_swr;
  int8_t level[NUM_BANDS];
  int8_t slevel[NUM_BANDS];
}
radio =
{
  0l,
  DEFAULT_STEP,
  DEFAULT_FREQUENCY,
  DEFAULT_BAND,
  DEFAULT_CW_SPEED,
  DEFAULT_CW_LEVEL,
  DEFAULT_SIDETONE,
  (uint32_t)(((uint64_t)DEFAULT_SIDETONE * (1ull << 32)) / SAMPLERATE),
  0l,
  SPECTRUM_WIND,
  JNR_OFF,
  NB_OFF,
  DEFAULT_MICGAIN,
  DEFAULT_MICPROC,
  DEFAULT_BANDWIDTH,
  DEFAULT_MODE,
  false,
  false,
  false,
  false,
  false,
  true,
  false,
  true,
  false,
  {0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0}
};

static struct
{
  const uint32_t lo;
  const uint32_t hi;
  uint32_t frequency;
}
bands[] =
{
  { 3500000UL,  3800000UL,  3600000UL},
  { 7000000UL,  7300000UL,  7100000UL},
  {10100000UL, 10150000UL, 10120000UL},
  {14000000UL, 14350000UL, 14200000UL},
  {18068000UL, 18168000UL, 18100000UL},
  {21000000UL, 21450000UL, 21200000UL},
  {24890000UL, 24990000UL, 24900000UL},
  {28000000UL, 29700000UL, 28500000UL},
  { 3500000UL, 30000000UL,  5000000UL}
};

volatile static struct
{
  uint32_t frequency;
}
save_data[] =
{
  { 3600000UL},
  { 7100000UL},
  {10120000UL},
  {14200000UL},
  {18100000UL},
  {21200000UL},
  {24900000UL},
  {28500000UL},
  { 5000000UL}
};

volatile static struct
{
  uint32_t usage;
  uint32_t count;
} cpu = {0,0};

Si5351 SI5351;
MCP3021 fwdADC;
MCP3021 refADC;
TCA9534 lpf_port;
TCA9534 bpf_port;
Rotary r = Rotary(PIN_ENCB,PIN_ENCA);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite lcd = TFT_eSprite(&tft);
I2S i2s(INPUT);

auto_init_mutex(rotary_mutex);
auto_init_mutex(cw_decode_mutex);
volatile static uint32_t audio_pwm = 0;
volatile static uint32_t tx_i_pwm = 0;
volatile static uint32_t tx_q_pwm = 0;
volatile static int32_t dac_h = 0;
volatile static int32_t dac_l = 0;
volatile static int32_t dac_value_i_p = 0;
volatile static int32_t dac_value_i_n = 0;
volatile static int32_t dac_value_q_p = 0;
volatile static int32_t dac_value_q_n = 0;
volatile static int16_t adc_value = 0;
volatile static bool adc_value_ready = false;
volatile static uint32_t adc_sample_p = 0;
volatile static uint32_t ft8_sample_p = 0;
volatile static uint32_t mic_peak_level = 0;
volatile static int16_t adc_data_i[MAX_ADC_SAMPLES] = {0};
volatile static int16_t adc_data_q[MAX_ADC_SAMPLES] = {0};
volatile static float ft8_data[FT8_FFT_SIZE] = {0.0f};
volatile static float mic_gain = 0.0f;
volatile static bool dit_latched = false;
volatile static bool dah_latched = false;
volatile static bool save_settings_now = false;
volatile static bool setup_complete = false;
volatile static bool set_spectrum_level = false;
volatile static bool adj_spectrum_level = false;
volatile static bool vox_triggered = false;
volatile static bool vox_mic_ready = false;
volatile static char cw_decode_buf[32] = "";

volatile static uint32_t wp = 0;
static uint8_t water[WATERFALL_ROWS][LCD_WIDTH] = {0};
static uint8_t magnitude[1024] = {0};

static void error_stop(const uint32_t err_code)
{
  // self-test error code
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  for (;;)
  {
    for (uint32_t i=0;i<err_code;i++)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN,LOW);
      delay(250);
    }
    delay(1000);
  }
}

static void mute(void)
{
  bpf_port.output(I2C_PIN_MUTE,TCA9534::Level::L);
}

static void unmute(void)
{
  bpf_port.output(I2C_PIN_MUTE,TCA9534::Level::H);
}

static void set_filter(void)
{
  // BPF
  static uint8_t old_bpf = 0;
  uint8_t new_bpf = 0;
  if (radio.frequency<5000000UL)
  {
    // BPF 3MHz-5MHz
    new_bpf = 1;
  }
  else if (radio.frequency<8000000UL)
  {
    // BPF 4MHz-8MHz
    new_bpf = 2;
  }
  else if (radio.frequency<16000000UL)
  {
    // BPF 8MHz-16MHz
    new_bpf = 3;
  }
  else
  {
    // BPF 16MHz-30MHz
    new_bpf = 4;
  }
  if (old_bpf != new_bpf)
  {
    old_bpf = new_bpf;
    switch (new_bpf)
    {
      case 1:
      {
        // BPF 3MHz-5MHz
        bpf_port.output(I2C_PIN_BPF1,TCA9534::Level::H);
        bpf_port.output(I2C_PIN_BPF2,TCA9534::Level::H);
        break;
      }
      case 2:
      {
        // BPF 4MHz-8MHz
        bpf_port.output(I2C_PIN_BPF1,TCA9534::Level::L);
        bpf_port.output(I2C_PIN_BPF2,TCA9534::Level::H);
        break;
      }
      case 3:
      {
        // BPF 8MHz-16MHz
        bpf_port.output(I2C_PIN_BPF1,TCA9534::Level::H);
        bpf_port.output(I2C_PIN_BPF2,TCA9534::Level::L);
        break;
      }
      case 4:
      {
        // BPF 16MHz-30MHz
        bpf_port.output(I2C_PIN_BPF1,TCA9534::Level::L);
        bpf_port.output(I2C_PIN_BPF2,TCA9534::Level::L);
        break;
      }
    }
  }

  // LPF
  static uint8_t old_lpf = 0;
  uint8_t new_lpf = I2C_PIN_LPF4;
  if (radio.frequency<6000000UL)
  {
    new_lpf = I2C_PIN_LPF1;
  }
  else if (radio.frequency<11000000UL)
  {
    new_lpf = I2C_PIN_LPF2;
  }
  else if (radio.frequency<20000000UL)
  {
    new_lpf = I2C_PIN_LPF3;
  }
  if (old_lpf != new_lpf)
  {
    old_lpf = new_lpf;
    lpf_port.output(I2C_PIN_LPF1,TCA9534::Level::L);
    lpf_port.output(I2C_PIN_LPF2,TCA9534::Level::L);
    lpf_port.output(I2C_PIN_LPF3,TCA9534::Level::L);
    lpf_port.output(I2C_PIN_LPF4,TCA9534::Level::L);
    lpf_port.output(new_lpf,TCA9534::Level::H);
  }
}

static __attribute__((always_inline)) inline void display_clear(void)
{
  lcd.fillSprite(LCD_BLACK);
}

static __attribute__((always_inline)) inline void display_refresh(void)
{
  lcd.pushSprite(0,0);
}

static __attribute__((always_inline)) inline void enable_mic(void)
{
  adc_gpio_init(PIN_MIC);
  adc_select_input(MIC_MUX);
}

static __attribute__((always_inline)) inline void disable_mic(void)
{
  pinMode(PIN_MIC,OUTPUT);
  digitalWrite(PIN_MIC,LOW);
}

void __not_in_flash_func(adc_interrupt_handler)(void)
{
  volatile static uint32_t counter = 0;
  volatile static uint32_t adc_raw = 0;
  if (adc_fifo_get_level()<4u)
  {
    return;
  }
  adc_raw += adc_fifo_get();
  adc_raw += adc_fifo_get();
  adc_raw += adc_fifo_get();
  adc_raw += adc_fifo_get();
  if (counter==4)
  {
    pwm_set_both_levels(audio_pwm,dac_l,dac_h);
    pwm_set_both_levels(tx_i_pwm,dac_value_i_p,dac_value_i_n);
    pwm_set_both_levels(tx_q_pwm,dac_value_q_p,dac_value_q_n);
    adc_value = (int16_t)(adc_raw>>4)-2048;
    adc_value_ready = true;
    adc_raw = 0;
    counter = 0;
  }
  counter++;
}

static void init_adc(void)
{
  adc_init();
  adc_gpio_init(PIN_MIC);
  adc_select_input(MIC_MUX);
  adc_fifo_setup(true, false, 4, false, false);
  adc_fifo_drain();
  adc_irq_set_enabled(true);
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_interrupt_handler);
  irq_set_priority(ADC_IRQ_FIFO, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_run(true);
}

static void init_i2s(void)
{
  // Note: LRCLK = BCLK + 1
  i2s.setDATA(PIN_DOUT);
  i2s.setBCLK(PIN_BCLK);
  i2s.setMCLK(PIN_MCLK);
  i2s.setBitsPerSample(32);
  i2s.setFrequency(SAMPLERATE);
  i2s.setMCLKmult(256);
  i2s.setBuffers(4, 256, 0);
  i2s.begin();
}

static void save_settings(void)
{
  static const uint32_t key = 0x12345678ul;
  mute();
  delay(100);
  i2s.end();
  EEPROM.begin(256);
  EEPROM.put(0x0*sizeof(uint32_t),key);
  EEPROM.put(0x1*sizeof(uint32_t),(uint32_t)radio.cw_dit);
  EEPROM.put(0x2*sizeof(uint32_t),(uint32_t)radio.cw_level);
  EEPROM.put(0x3*sizeof(uint32_t),(uint32_t)radio.sidetone);
  EEPROM.put(0x4*sizeof(uint32_t),(uint32_t)radio.cw_phase);
  EEPROM.put(0x5*sizeof(uint32_t),(uint32_t)radio.spectype);
  EEPROM.put(0x6*sizeof(uint32_t),(uint32_t)radio.jnrlevel);
  EEPROM.put(0x7*sizeof(uint32_t),(uint32_t)radio.micgain);
  EEPROM.put(0x8*sizeof(uint32_t),(uint32_t)radio.cessb?1u:0u);
  EEPROM.put(0x9*sizeof(uint32_t),(uint32_t)radio.bandwidth);
  EEPROM.put(0xa*sizeof(uint32_t),(uint32_t)radio.graph_swr?1u:0u);
  for (uint32_t i=0;i<NUM_BANDS;i++)
  {
    EEPROM.put((i+0xb)*sizeof(uint32_t),(uint32_t)radio.level[i]);
  }
  EEPROM.end();
  init_i2s();
  unmute();
}

static void restore_settings(void)
{
  uint32_t key = 0 ;
  EEPROM.begin(256);
  EEPROM.get(0,key);
  if (key==0x12345678)
  {
    uint32_t data32 = 0;
    EEPROM.get(0x1*sizeof(uint32_t),data32); radio.cw_dit   = data32;
    EEPROM.get(0x2*sizeof(uint32_t),data32); radio.cw_level = data32;
    EEPROM.get(0x3*sizeof(uint32_t),data32); radio.sidetone = data32;
    EEPROM.get(0x4*sizeof(uint32_t),data32); radio.cw_phase = data32;
    EEPROM.get(0x5*sizeof(uint32_t),data32); radio.spectype = data32;
    EEPROM.get(0x6*sizeof(uint32_t),data32); radio.jnrlevel = data32;
    EEPROM.get(0x7*sizeof(uint32_t),data32); radio.micgain = data32;
    EEPROM.get(0x8*sizeof(uint32_t),data32); radio.cessb = data32==1?true:false;
    EEPROM.get(0x9*sizeof(uint32_t),data32); radio.bandwidth = data32;
    EEPROM.get(0xa*sizeof(uint32_t),data32); radio.graph_swr = data32==1?true:false;
    for (uint32_t i=0;i<NUM_BANDS;i++)
    {
      EEPROM.get((i+0xb)*sizeof(uint32_t),data32);
      radio.level[i] = (int8_t)data32;
      if (radio.level[i]<SPECTRUM_LEVEL_MIN || radio.level[i]>SPECTRUM_LEVEL_MAX)
      {
        radio.level[i] = 0;
      }
      radio.slevel[i] = radio.level[i];
    }
  }
  if (radio.micgain<25ul || radio.micgain>200ul)
  {
    radio.micgain = DEFAULT_MICGAIN;
  }
  if (radio.bandwidth<1ul || radio.bandwidth>5ul)
  {
    radio.bandwidth = DEFAULT_BANDWIDTH;
  }
  EEPROM.end();
}

static void set_frequency(const uint32_t ft8_offset = 0u)
{
  volatile static uint32_t old_quadrature_divisor = 0u;
  const uint32_t quadrature_divisor = UTIL::quadrature_divisor(radio.frequency);
  const int32_t ssboffset = radio.tx_enable?0:488;
  const int32_t cwoffset = radio.tx_enable?0:radio.sidetone;
  int32_t rx_ssboffset = 0;
  int32_t rx_cwoffsetw = 0;
  switch (radio.mode)
  {
    case MODE_LSB: rx_ssboffset = +ssboffset;      break;
    case MODE_USB: rx_ssboffset = -ssboffset;      break;
    case MODE_DGL: rx_ssboffset = +ssboffset;      break;
    case MODE_DGU: rx_ssboffset = -ssboffset;      break;
    case MODE_FT8: rx_ssboffset = -ssboffset;      break;
    case MODE_AM:  rx_ssboffset = -(SAMPLERATE/4); break;
    case MODE_CWL: rx_cwoffsetw = +cwoffset;       break;
    case MODE_CWU: rx_cwoffsetw = -cwoffset;       break;
  }
  const uint64_t f = SI5351_FREQ_MULT * (radio.frequency+rx_ssboffset+rx_cwoffsetw) + ft8_offset;
  const uint64_t p = f * quadrature_divisor;
  SI5351.set_freq_manual(f,p,SI5351_CLK0);
  SI5351.set_freq_manual(f,p,SI5351_CLK1);
  if (old_quadrature_divisor != quadrature_divisor)
  {
    old_quadrature_divisor = quadrature_divisor;
    SI5351.set_phase(SI5351_CLK0,0);
    SI5351.set_phase(SI5351_CLK1,quadrature_divisor);
    SI5351.pll_reset(SI5351_PLLA);
  }
}

void setup(void)
{
  // run DSP on core 0
  pinMode(PIN_REG,OUTPUT);
  // set pico regulator to low noise
  digitalWrite(PIN_REG,HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  const uint32_t clksys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  // frequency_count_khz() isn't accurate!
  if (clksys < (240000ul - 5ul) || clksys > (240000ul + 5ul))
  {
    // trap the wrong system clock
    error_stop(ERROR_SYSCLOCK);
  }
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_TXBIAS,OUTPUT);
  pinMode(PIN_PTT,INPUT_PULLUP);
  pinMode(PIN_PADA,INPUT_PULLUP);
  pinMode(PIN_PADB,INPUT_PULLUP);
  pinMode(PIN_ENCBUT,INPUT_PULLUP);

  // ensure TX bias is off
  digitalWrite(PIN_TXBIAS,LOW);

  // high speed I2C
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(400000ul);
  Wire.begin();

  // filters and TX/RX control
  lpf_port.attach(Wire);
  if (!lpf_port.setDeviceAddress(LPF_I2C_ADDRESS))
  {
    error_stop(ERROR_LPF);
  }
  bpf_port.attach(Wire);
  if (!bpf_port.setDeviceAddress(BPF_I2C_ADDRESS))
  {
    error_stop(ERROR_BPF);
  }
  lpf_port.config(TCA9534::Config::OUT);
  lpf_port.polarity(TCA9534::Polarity::ORIGINAL);
  bpf_port.config(TCA9534::Config::OUT);
  bpf_port.polarity(TCA9534::Polarity::ORIGINAL);
  lpf_port.output(I2C_PIN_LPF1,TCA9534::Level::L);
  lpf_port.output(I2C_PIN_LPF2,TCA9534::Level::L);
  lpf_port.output(I2C_PIN_LPF3,TCA9534::Level::L);
  lpf_port.output(I2C_PIN_LPF4,TCA9534::Level::L);
  lpf_port.output(I2C_PIN_TXENABLE,TCA9534::Level::L);
  bpf_port.output(I2C_PIN_MUTE,TCA9534::Level::L);
  bpf_port.output(I2C_PIN_BPF1,TCA9534::Level::L);
  bpf_port.output(I2C_PIN_BPF2,TCA9534::Level::H);
  bpf_port.output(I2C_PIN_TXN,TCA9534::Level::H);
  bpf_port.output(I2C_PIN_RXN,TCA9534::Level::L);
  delay(40);
  digitalWrite(LED_BUILTIN,LOW);

  // TX unsafe if PTT enabled at start up
  if (digitalRead(PIN_PTT)==HIGH && digitalRead(PIN_PADA)==HIGH && digitalRead(PIN_PADB)==HIGH)
  {
    radio.tx_safe = true;
  }

  // set pin function to PWM
  gpio_set_function(PIN_TX000,GPIO_FUNC_PWM); //  6  PWM
  gpio_set_function(PIN_TX180,GPIO_FUNC_PWM); //  8  PWM
  gpio_set_function(PIN_TX090,GPIO_FUNC_PWM); // 10  PWM
  gpio_set_function(PIN_TX270,GPIO_FUNC_PWM); // 12  PWM

  // get PWM slice connected to each pin pair
  tx_i_pwm = pwm_gpio_to_slice_num(PIN_TX000);
  tx_q_pwm = pwm_gpio_to_slice_num(PIN_TX090);
  
  // set period of 1024 cycles
  pwm_set_wrap(tx_i_pwm,1023);
  pwm_set_wrap(tx_q_pwm,1023);
  
  // initialise to zero (low)
  pwm_set_both_levels(tx_i_pwm,0,0);
  pwm_set_both_levels(tx_q_pwm,0,0);

  // set each PWM running
  pwm_set_enabled(tx_i_pwm,true);
  pwm_set_enabled(tx_q_pwm,true);

  // set up audio out PWM
  gpio_set_function(PIN_AUPWMH,GPIO_FUNC_PWM);
  gpio_set_function(PIN_AUPWML,GPIO_FUNC_PWM);
  audio_pwm = pwm_gpio_to_slice_num(PIN_AUPWML);
  pwm_set_wrap(audio_pwm,63); // 240,000,000 / 64 = 3,750,000
  pwm_set_both_levels(audio_pwm,0,31);
  pwm_set_enabled(audio_pwm,true);

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  const bool si5351_found = SI5351.init(SI5351_CRYSTAL_LOAD_0PF,TCXO_FREQ,0);
  if (!si5351_found)
  {
    error_stop(ERROR_SI5351);
  }
  SI5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  SI5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_8MA);
  set_frequency();

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  if (!fwdADC.begin(0))
  {
    error_stop(ERROR_FWDADC);
  }
  if (!refADC.begin(1))
  {
    error_stop(ERROR_REFADC);
  }

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // init rotary
  r.begin();

  // init PLL and set default frequency
  radio.frequency = DEFAULT_FREQUENCY;
  radio.band = DEFAULT_BAND;
  radio.mode = DEFAULT_MODE;
  restore_settings();
  set_filter();

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // set up audio ADC (IQ input)
  init_i2s();

  // set up mic ADC
  init_adc();

  // disable Mic in RX mode
  disable_mic();

  // init LCD
  char sz_version[16] = "";
  char sz_clksys[16] = "";
  memset(sz_clksys,0,sizeof(sz_clksys));
  memset(sz_version,0,sizeof(sz_version));
  ultoa(clksys,sz_clksys,10);
  sz_clksys[3] = '\0';
  strcpy(sz_version,VERSION_STRING);
  strcat(sz_version,sz_clksys);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(LCD_BLACK);
  tft.displayOn();
  lcd.createSprite(LCD_WIDTH, LCD_HEIGHT);
  lcd.fillSprite(LCD_BLACK);
  lcd.pushSprite(0,0);
  for (uint32_t i=0;i<32;i++)
  {
    lcd.fillSprite(LCD_BLACK);
    lcd.setFreeFont(&FreeSansBold18pt7b);
    lcd.setTextSize(1);
    lcd.setTextColor(spectrum::color_map_32[i],LCD_BLACK);
    lcd.setCursor(POS_SPLASH_X,POS_SPLASH_Y);
    lcd.print("MBPTRX");
    lcd.setTextFont(1);
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_WHITE,LCD_BLACK);
    lcd.setCursor(POS_VERSION_X,POS_VERSION_Y);
    lcd.print(sz_version);
    lcd.pushSprite(0,0);
    delay(50);
  }
  delay(250);

  // intro screen
  lcd.fillSprite(LCD_BLACK);
  lcd.setTextColor(LCD_WHITE,LCD_BLACK);
  lcd.setCursor(POS_VERSION_X,POS_VERSION_Y);
  lcd.print(sz_version);
  lcd.setFreeFont(&FreeSansBold18pt7b);
  lcd.setCursor(POS_CALL_X,POS_SPLASH_Y);
  lcd.print(YOUR_CALL);
  lcd.pushSprite(0,0);
  lcd.setTextFont(1);

  for (uint8_t i=0;i<sizeof(cw_decode_buf);i++)
  {
    cw_decode_buf[i] = '\0';
  }

  // splash delay
  delay(2000);

  // unmute
  unmute();

  setup_complete = true;
}

void setup1(void)
{
  // run UI on core 1
  // only go to loop1 when setup() has completed
  while (!setup_complete)
  {
    tight_loop_contents();
  }
#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<5;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif
}

static void show_frequency(void)
{
  const uint16_t tx_rx = radio.tx_enable?LCD_RED:LCD_WHITE;
  const uint16_t fcolour = radio.tx_safe?tx_rx:LCD_YELLOW;
  char sz_frequency[16] = "";
  memset(sz_frequency,0,sizeof(sz_frequency));
  ultoa(radio.frequency,sz_frequency,10);
  if (radio.frequency<1000000u)
  {
    // 6 digits
    //            012 345
    // 936000 ->  936.000
    // 012345    01234567
    sz_frequency[7] = sz_frequency[5];
    sz_frequency[6] = sz_frequency[4];
    sz_frequency[5] = sz_frequency[3];
    sz_frequency[4] = '.';
    sz_frequency[3] = sz_frequency[2];
    sz_frequency[2] = sz_frequency[1];
    sz_frequency[1] = sz_frequency[0];
    sz_frequency[0] = ' ';
  }
  else if (radio.frequency<10000000u)
  {
    // 7 digits
    // 3555000 -> 3.555.000
    // 0123456    012345678
    sz_frequency[9] = sz_frequency[6];
    sz_frequency[8] = sz_frequency[5];
    sz_frequency[7] = sz_frequency[4];
    sz_frequency[6] = '.';
    sz_frequency[5] = sz_frequency[3];
    sz_frequency[4] = sz_frequency[2];
    sz_frequency[3] = sz_frequency[1];
    sz_frequency[2] = '.';
    sz_frequency[1] = sz_frequency[0];
    sz_frequency[0] = ' ';
  }
  else
  {
    // 8 digits
    // 14222000 -> 14.222.000
    // 01234567    0123456789
    sz_frequency[9] = sz_frequency[7];
    sz_frequency[8] = sz_frequency[6];
    sz_frequency[7] = sz_frequency[5];
    sz_frequency[6] = '.';
    sz_frequency[5] = sz_frequency[4];
    sz_frequency[4] = sz_frequency[3];
    sz_frequency[3] = sz_frequency[2];
    sz_frequency[2] = '.';
  }
  lcd.setFreeFont(&Arial_Bold16pt7b);
  lcd.setTextSize(1);
  lcd.setTextColor(fcolour,LCD_BLACK);
  lcd.setCursor(POS_FREQUENCY_X,POS_FREQUENCY_Y);
  lcd.print(sz_frequency);
  lcd.setTextFont(1);
}

static void show_tuning_step(void)
{
  if (radio.tx_enable)
  {
    return;
  }
  lcd.setTextSize(1);
  lcd.setCursor(POS_TUNING_STEP_X-30,POS_TUNING_STEP_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("STEP");
  lcd.setCursor(POS_TUNING_STEP_X,POS_TUNING_STEP_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print(radio.step);
}

static void show_swr(void)
{
  static const uint32_t PO_DECAY_RATE = 250ul;
  static const float A = 0.141570128f;
  static const float B = 5.64859373f;
  volatile static uint32_t max_po = 0; 
  volatile static uint32_t po_decay = 0;
  if (!radio.tx_enable)
  {
    max_po = 0;
    return;
  }
  const uint32_t adc_fwd_raw = (uint32_t)fwdADC.read();
  const uint32_t adc_ref_raw = (uint32_t)refADC.read();
  if (adc_fwd_raw==0xffffu || adc_ref_raw==0xffffu)
  {
    return;
  }

  // vswr
  uint32_t vswr = 100u;
  const float adcfwd = (float)(FILTER::ma1(adc_fwd_raw));
  const float adcref = (float)(FILTER::ma2(adc_ref_raw));
  const float vfwd = A * adcfwd + B;
  const float vref = A * adcref + (adcref>2.0f?B:0.0f);
  if (vfwd>vref)
  {
    vswr = (uint32_t)(100.0f * (vfwd + vref) / (vfwd - vref));
    vswr = constrain(vswr,100u,999u);
  }
  // power out
  // Vpp * Vpp / 400
  const uint32_t now = millis();
  const float fwd = A * (float)adc_fwd_raw + (adc_fwd_raw>2?B:0.0f);
  const uint32_t po = (uint32_t)(((fwd * fwd) / 400.0f) * 10.0f + 0.5f);
  if (po<max_po)
  {
    if (now>po_decay)
    {
      if (max_po>0) max_po--;
      po_decay = now + PO_DECAY_RATE;
    }
  }
  else
  {
    max_po = po;
    po_decay = now + PO_DECAY_RATE;
  }

  // show power and SWR
  if (radio.graph_swr)
  {
    // graph power
    const uint8_t p = max_po / 10u;
    for (uint8_t i=0;i<p;i++)
    {
      lcd.fillRect(POS_METER_X+i*5+0,POS_METER_Y+14,4,4,LCD_WHITE);
    }
    // graph SWR
    const uint8_t s = vswr / 100u;
    for (uint8_t i=0;i<s;i++)
    {
      lcd.fillRect(POS_METER_X+i*5+0,POS_METER_Y+20,4,4,LCD_WHITE);
    }
  }
  else
  {
    // display SWR
    char sz_swr[16] = "";
    memset(sz_swr,0,sizeof(sz_swr));
    ultoa(vswr,sz_swr,10);
    sz_swr[3] = sz_swr[2];
    sz_swr[2] = sz_swr[1];
    sz_swr[1] = '.';
    lcd.setTextSize(1);
    lcd.setCursor(POS_SWR_X-25,POS_SWR_Y);
    lcd.setTextColor(LCD_WHITE);
    lcd.print("SWR");
    lcd.setCursor(POS_SWR_X-5,POS_SWR_Y);
    lcd.setTextColor(LCD_WHITE);
    lcd.print(sz_swr);

    // display power
    char sz_po[16] = "";
    memset(sz_po,0,sizeof(sz_po));
    ultoa(max_po,sz_po,10);
    if (max_po<10)
    {
      sz_po[2] = sz_po[0];
      sz_po[1] = '.';
      sz_po[0] = '0';
    }
    else if (max_po<100)
    {
      sz_po[2] = sz_po[1];
      sz_po[1] = '.';
    }
    else
    {
      sz_po[3] = sz_po[2];
      sz_po[2] = '.';
    }
    lcd.setCursor(POS_SWR_X+25,POS_SWR_Y);
    lcd.print("PO");
    lcd.setCursor(POS_SWR_X+40,POS_SWR_Y);
    lcd.print(sz_po);
  }
}

static void show_tx(void)
{
  lcd.setTextSize(1);
  lcd.setFreeFont(&FreeSansBold9pt7b);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(LCD_BLUE);
  lcd.print("RX");
  lcd.setTextFont(1);
}

static void show_rx(void)
{
  lcd.setTextSize(1);
  lcd.setFreeFont(&FreeSansBold9pt7b);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(LCD_BLUE);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("RX");
  lcd.setTextFont(1);
}

static void show_swl(void)
{
  lcd.setTextSize(1);
  lcd.setFreeFont(&FreeSansBold9pt7b);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(radio.tx_button?LCD_RED:LCD_GREEN);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(radio.tx_button?LCD_RED:LCD_GREEN);
  lcd.print("RX");
  lcd.setTextFont(1);
}

static void show_rx_tx(void)
{
  if ((radio.band==BAND_SWL) || (radio.mode==MODE_AM))
  {
    show_swl();
  }
  else if (radio.tx_enable)
  {
    show_tx();
  }
  else
  {
    show_rx();
  }
}

static void show_mode(void)
{
  lcd.fillRect(POS_MODE_X-5,POS_MODE_Y-5,45,25,LCD_WHITE);
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_BLACK);
  lcd.setCursor(POS_MODE_X,POS_MODE_Y);
  const char *sz_mode = "XXX";
  if (radio.tx_enable)
  {
    // no TX in AM
    switch (radio.mode)
    {
      case MODE_LSB: sz_mode = "LSB"; break;
      case MODE_USB: sz_mode = "USB"; break;
      case MODE_CWL: sz_mode = "CWL"; break;
      case MODE_CWU: sz_mode = "CWU"; break;
      case MODE_DGL: sz_mode = "DGL"; break;
      case MODE_DGU: sz_mode = "DGU"; break;
      case MODE_FT8: sz_mode = "FT8"; break;
    }
  }
  else
  {
    switch (radio.mode)
    {
      case MODE_LSB: sz_mode = "LSB"; break;
      case MODE_USB: sz_mode = "USB"; break;
      case MODE_CWL: sz_mode = "CWL"; break;
      case MODE_CWU: sz_mode = "CWU"; break;
      case MODE_DGL: sz_mode = "DGL"; break;
      case MODE_DGU: sz_mode = "DGU"; break;
      case MODE_FT8: sz_mode = "FT8"; break;
      case MODE_AM:  sz_mode = "AM";  break;
    }
  }
  lcd.print(sz_mode);
}

static void show_band(void)
{
  const uint32_t background = (radio.band==BAND_SWL)?LCD_RED:LCD_PURPLE;
  lcd.fillRect(POS_BAND_X-5,POS_BAND_Y-5,45,25,background);
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(POS_BAND_X,POS_BAND_Y);
  const char *sz_band = "";
  switch (radio.band)
  {
    case BAND_80M: sz_band = "80M"; break;
    case BAND_40M: sz_band = "40M"; break;
    case BAND_30M: sz_band = "30M"; break;
    case BAND_20M: sz_band = "20M"; break;
    case BAND_17M: sz_band = "17M"; break;
    case BAND_15M: sz_band = "15M"; break;
    case BAND_12M: sz_band = "12M"; break;
    case BAND_10M: sz_band = "10M"; break;
    case BAND_SWL: sz_band = "SWL"; break;
  }
  lcd.print(sz_band);
}

static void show_cpu_usage(void)
{
  static uint32_t cpu_usage = 0;
  static uint32_t next_update = 0;
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update = now + 1000ul;
    const uint32_t usage = cpu.usage;
    const uint32_t count = cpu.count;
    cpu.usage = 0;
    cpu.count = 0;
    cpu_usage = (100u * usage / count) >> 5;
  }
  lcd.fillRect(POS_CPU_X-5,POS_CPU_Y-5,45,25,LCD_RED);
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(POS_CPU_X,POS_CPU_Y);
  lcd.print(cpu_usage);
  if (cpu_usage<100)
  {
    lcd.print("%");
  }
}

static void show_jnr(void)
{
  if (radio.jnrlevel==0)
  {
    return;
  }
  if (radio.mode==MODE_DGU || radio.mode==MODE_DGL || radio.mode==MODE_FT8)
  {
    // no JNR for digital modes
    return;
  }
  lcd.fillRect(POS_JNR_X-5,POS_JNR_Y-1,45,10,LCD_PURPLE);
  lcd.setTextSize(1);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(POS_JNR_X,POS_JNR_Y);
  switch (radio.jnrlevel)
  {
    case 1u: lcd.print("JNR:1"); break;
    case 2u: lcd.print("JNR:2"); break;
    case 3u: lcd.print("JNR:3"); break;
  }
}

static void show_debug_value(const int32_t v)
{
  if (v==0)
  {
    return;
  }
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(POS_DEBUG_X,POS_DEBUG_Y);
  lcd.print(v);
}

static void show_meter_dial(const uint8_t sig)
{
  const uint8_t v = min(sig,15);
  lcd.setTextSize(1);
  lcd.setCursor(POS_METER_X,POS_METER_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("1 3 5 7 9 +20");
  for (uint8_t i=0;i<v;i++)
  {
    lcd.fillRect(POS_METER_X+i*5+0,POS_METER_Y+8,4,4,LCD_WHITE);
  }
}

static void show_cw_settings(void)
{
  if (set_spectrum_level || adj_spectrum_level)
  {
    return;
  }
  if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
  {
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_YELLOW);
    lcd.setCursor(POS_CW_SETTINGS_X,POS_CW_SETTINGS_Y);
    const char *sz_wpm = "";
    switch (radio.cw_dit)
    {
      case 120u: sz_wpm = "10"; break;
      case 80u:  sz_wpm = "15"; break;
      case 60u:  sz_wpm = "20"; break;
      case 48u:  sz_wpm = "25"; break;
      case 40u:  sz_wpm = "30"; break;
    }
    lcd.print("WPM:");
    lcd.print(sz_wpm);
    const char *sz_level = "";
    switch (radio.cw_level)
    {
      case 1u: sz_level = "LOW"; break;
      case 2u: sz_level = "MED"; break;
      case 3u: sz_level = "HI";  break;
    }
    lcd.print(" Level:");
    lcd.print(sz_level);
    lcd.print(" Sidetone:");
    lcd.print(radio.sidetone);
    if (radio.cwdecode!=0)
    {
      lcd.drawRect(176,POS_CW_SETTINGS_Y-2,12,11,LCD_RED);
      lcd.setCursor(179,POS_CW_SETTINGS_Y);
      lcd.print(radio.cwdecode==1?"A":"S");
    }
  }
}

static void show_cessb_settings(void)
{
  if (set_spectrum_level || adj_spectrum_level)
  {
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_GREEN);
    lcd.setCursor(POS_CESSB_MIC_X,POS_CESSB_MIC_Y);
    lcd.print("Spectrum Level: ");
    lcd.print(radio.level[radio.band]*6);
    lcd.print("dB");
  }
  else
  {
    if (radio.mode==MODE_FT8)
    {
      lcd.setTextSize(1);
      lcd.setTextColor(LCD_GREEN);
      lcd.setCursor(POS_CESSB_MIC_X,POS_CESSB_MIC_Y);
      lcd.print("AGC:Off");
      lcd.print(" BW:3000Hz");
      lcd.print(" CESSB:Off");
      lcd.print(" NB:Off");
    }
    else if (radio.mode==MODE_DGU || radio.mode==MODE_DGL)
    {
      lcd.setTextSize(1);
      lcd.setTextColor(LCD_GREEN);
      lcd.setCursor(POS_CESSB_MIC_X,POS_CESSB_MIC_Y);
      lcd.print("Vox:On");
      lcd.print(" BW:3000Hz");
      lcd.print(" CESSB:Off");
      lcd.print(" NB:Off");
    }
    else if (radio.mode==MODE_LSB || radio.mode==MODE_USB)
    {
      lcd.setTextSize(1);
      lcd.setTextColor(LCD_GREEN);
      lcd.setCursor(POS_CESSB_MIC_X,POS_CESSB_MIC_Y);
      lcd.print("CESSB:");
      lcd.print(radio.cessb?"On":"Off");
      lcd.setCursor(POS_CESSB_MIC_X+58,POS_CESSB_MIC_Y);
      if (radio.micproc==0)
      {
        lcd.print("Mic:");
        lcd.print(radio.micgain);
        lcd.print("%");
      }
      else
      {
        lcd.setTextColor(LCD_RED);
        lcd.print("Mic:");
        lcd.print(radio.micproc);
        lcd.print("P");
        lcd.setTextColor(LCD_GREEN);
      }
      lcd.setCursor(POS_CESSB_MIC_X+110,POS_CESSB_MIC_Y);
      lcd.print("BW:");
      switch (radio.bandwidth)
      {
        case 1: lcd.print("2000Hz"); break;
        case 2: lcd.print("2200Hz"); break;
        case 3: lcd.print("2400Hz"); break;
        case 4: lcd.print("2600Hz"); break;
        case 5: lcd.print("2800Hz"); break;
      }
      lcd.setCursor(POS_CESSB_MIC_X+168,POS_CESSB_MIC_Y);
      switch (radio.nblevel)
      {
        case 0: lcd.print("NB0"); break;
        case 1: lcd.print("NB1"); break;
        case 2: lcd.print("NB2"); break;
        case 3: lcd.print("NB3"); break;
        case 4: lcd.print("NB4"); break;
        case 5: lcd.print("NB5"); break;
      }
    }
  }
}

static void show_cw_decode(void)
{
  static char z_buf[32] = "";
  if (radio.cwdecode==0)
  {
    return;
  }
  if (!(radio.mode==MODE_CWU || radio.mode==MODE_CWL))
  {
    return;
  }
  if (!mutex_try_enter(&cw_decode_mutex,0ul))
  {
    return;
  }
  strcpy(z_buf,(char*)cw_decode_buf);
  mutex_exit(&cw_decode_mutex);
  lcd.fillRect(0,115,240,20,LCD_BLACK);
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(0,116);
  lcd.print(z_buf);
}

static void show_spectrum(void)
{
  // show the spectrum
  static const int32_t LSB_OFFSET = -5;
  static const int32_t USB_OFFSET = +5;
  static const int32_t AM_OFFSET = +45;
  lcd.setTextSize(1);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(0,POS_WATER_Y+4);
  lcd.print("-14KHz");
  lcd.setCursor(LCD_WIDTH-38,POS_WATER_Y+4);
  lcd.print("+14KHz");

  if (radio.tx_enable)
  {
    switch (radio.mode)
    {
      case MODE_LSB:
      {
        for (uint32_t x=0;x<20;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT-x,POS_WATER_Y,POS_CENTER_RIGHT-x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_USB:
      {
        for (uint32_t x=0;x<20;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_DGL:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT-x,POS_WATER_Y,POS_CENTER_RIGHT-x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_DGU:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_FT8:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_CWL:
      {
        for (uint32_t x=0;x<5;x++)
        {
          lcd.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_CWU:
      {
        for (uint32_t x=0;x<5;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
    }
  }
  else
  {
    // receive
    switch (radio.mode)
    {
      case MODE_LSB:
      {
        for (uint32_t x=0;x<20;x++)
        {
          lcd.drawLine(POS_CENTER_LEFT-x+LSB_OFFSET,POS_WATER_Y,POS_CENTER_LEFT-x+LSB_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_USB:
      {
        for (uint32_t x=0;x<20;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y,POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_DGL:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_LEFT-x+LSB_OFFSET,POS_WATER_Y,POS_CENTER_LEFT-x+LSB_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_DGU:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y,POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_FT8:
      {
        for (uint32_t x=0;x<25;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y,POS_CENTER_RIGHT+x+USB_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_AM:
      {
        for (uint32_t x=0;x<40;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x+AM_OFFSET,POS_WATER_Y,POS_CENTER_RIGHT+x+AM_OFFSET,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_CWL:
      {
        for (uint32_t x=0;x<10;x++)
        {
          lcd.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
      case MODE_CWU:
      {
        for (uint32_t x=0;x<10;x++)
        {
          lcd.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,LCD_MODE);
        }
        break;
      }
    }
  }

  // greatest of 4 adjacent magnitudes
  for (uint32_t x=0;x<LCD_WIDTH;x++)
  {
    // x=0 => x*4+33 = 0*4+33 = 33
    // x=0 => x*4+34 = 0*4+34 = 34
    // x=0 => x*4+35 = 0*4+35 = 35
    // x=0 => x*4+36 = 0*4+36 = 36
    // x=1 => x*4+33 = 1*4+33 = 37
    // x=1 => x*4+34 = 1*4+34 = 38
    // x=1 => x*4+35 = 1*4+35 = 39
    // x=1 => x*4+36 = 1*4+36 = 40
    // ...
    // x=238 => x*4+33 = 238*4+33 = 985
    // x=238 => x*4+34 = 238*4+34 = 986
    // x=238 => x*4+35 = 238*4+35 = 987
    // x=238 => x*4+36 = 238*4+36 = 988
    // x=239 => x*4+33 = 239*4+33 = 989
    // x=239 => x*4+34 = 239*4+34 = 990
    // x=239 => x*4+35 = 239*4+35 = 991
    // x=239 => x*4+36 = 239*4+36 = 992
    uint8_t droplet = magnitude[x*4+33];
    droplet = max(droplet,magnitude[x*4+34]);
    droplet = max(droplet,magnitude[x*4+35]);
    droplet = max(droplet,magnitude[x*4+36]);
    if (droplet>31) droplet = 31;
    water[wp][x] = droplet;
  }

  // draw the spectrum
  if (radio.spectype==SPECTRUM_WIND)
  {
    for (uint32_t x=0;x<LCD_WIDTH-1;x++)
    {
      const int32_t v0 = water[wp][x];
      const int32_t v1 = water[wp][x+1];
      const int32_t x0 = x;
      const int32_t y0 = POS_WATER_Y+31-v0;
      const int32_t x1 = x+1;
      const int32_t y1 = POS_WATER_Y+31-v1;
      lcd.drawLine(x0,y0,x1,y1,LCD_WHITE);
    }
  }
  else
  {
    // SPECTRUM_GRASS
    lcd.fillRectVGradient(0,POS_WATER_Y,240,32,LCD_RED,LCD_YELLOW);
    for (uint32_t x=0;x<LCD_WIDTH;x++)
    {
      /*
      // single colour grass
      const int32_t x0 = x;
      const int32_t y0 = POS_WATER_Y+31;
      const int32_t x1 = x;
      const int32_t y1 = POS_WATER_Y+31-water[wp][x];
      lcd.drawLine(x0,y0,x1,y1,LCD_WHITE);
      */
      const int32_t x0 = x;
      const int32_t y0 = POS_WATER_Y;
      const int32_t x1 = x;
      const int32_t y1 = POS_WATER_Y+30-water[wp][x];
      lcd.drawLine(x0,y0,x1,y1,LCD_BLACK);
    }
  }

  // draw the waterfall
  int32_t r = wp;
  int32_t y = POS_WATER_Y+32;
  for (uint32_t i=0;i<WATERFALL_ROWS;i++,y++)
  {
    for (uint32_t x=0;x<LCD_WIDTH;x++)
    {
      const uint16_t c = spectrum::color_map_32[water[r][x]];
      lcd.drawPixel(x,y,c);
    }
    r--;
    if (r<0) r = WATERFALL_ROWS-1;
  }
  wp++;
  if (wp>=WATERFALL_ROWS) wp = 0;
}

enum menu_button_state_t {MENU_BUTTON_IDLE,MENU_BUTTON_PRESSED,MENU_WAIT_RELEASE_0,MENU_WAIT_RELEASE_1};
enum menu_button_action_t {MENU_NONE,MENU_TOP_SELECT,MENU_OPTION_SELECT};
static menu_button_state_t menu_button_state = MENU_BUTTON_IDLE;
static menu_button_action_t menu_button_action = MENU_NONE;
static uint8_t menu_window = 0;
static uint8_t menu_current = 0;
static uint8_t option_window = 0;
static uint8_t option_current = 0;

static void show_menu(void)
{
  if (!radio.menu_active)
  {
    return;
  }
  lcd.fillRect(POS_MENU_X,POS_MENU_Y,MENU_WIDTH,MENU_HEIGHT,LCD_BLACK);
  lcd.drawRect(POS_MENU_X,POS_MENU_Y,MENU_WIDTH,MENU_HEIGHT,LCD_WHITE),
  lcd.setTextSize(1);
  lcd.setFreeFont(&FreeSansBold9pt7b);
  if (menu_button_action==MENU_NONE) // MENU_MENU_SELECT
  {
    // display top level menu
    for (uint8_t i=menu_window,j=0;i<NUM_MENU_ITEMS && j<4;i++,j++)
    {
      char sz_menu_name[16] = "";
      memset(sz_menu_name,' ',sizeof(sz_menu_name));
      sz_menu_name[10] = '\0';
      for (int k=0;k<10;k++)
      {
        const char c = menu_options[i].menu_name[k];
        if (c=='\0') break;
        sz_menu_name[k] = c;
      }
      if (i==menu_current)
      {
        lcd.setTextColor(LCD_BLACK);
        lcd.fillRectHGradient(POS_MENU_X+2,POS_MENU_Y+20*j+1,MENU_WIDTH-4,20,LCD_YELLOW,LCD_PINK);
      }
      else
      {
        lcd.setTextColor(LCD_WHITE);
      }
      lcd.setCursor(POS_MENU_X+5,POS_MENU_Y+20*j+16);
      lcd.print(sz_menu_name);
    }
  }
  else if (menu_button_action==MENU_TOP_SELECT) // MENU_OPTION_SELECT
  {
    if (menu_current<NUM_MENU_ITEMS)
    {
      const uint8_t num_options = menu_options[menu_current].num_options;
      if (option_window<num_options && option_current<num_options)
      {
        lcd.setCursor(POS_MENU_X+5,POS_MENU_Y+16);
        lcd.print(menu_options[menu_current].menu_name);
        for (uint8_t i=option_window,j=1;i<num_options && j<4;i++,j++)
        {
          char sz_option_name[16] = "";
          memset(sz_option_name,' ',sizeof(sz_option_name));
          sz_option_name[10] = '\0';
          for (int k=1;k<10;k++)
          {
            const char c = menu_options[menu_current].options[i].option_name[k-1];
            if (c=='\0') break;
            sz_option_name[k] = c;
          }
          if (i==option_current)
          {
            lcd.setTextColor(LCD_BLACK);
            lcd.fillRectHGradient(POS_MENU_X+2,POS_MENU_Y+20*j,MENU_WIDTH-4,20,LCD_YELLOW,LCD_PINK);
          }
          else
          {
            lcd.setTextColor(LCD_WHITE);
          }
          lcd.setCursor(POS_MENU_X+10,POS_MENU_Y+20*j+16);
          lcd.print(sz_option_name);
        }
      }
    }
  }
  lcd.setTextFont(1);
}

static void update_display(const uint32_t signal_level = 0u,const int32_t debug_value = 0)
{
  display_clear();
  show_rx_tx();
  show_mode();
  show_band();
  show_cpu_usage();
  show_frequency();
  show_tuning_step();
  show_swr();
  show_meter_dial(signal_level);
  show_cw_settings();
  show_cessb_settings();
  show_jnr();
  show_spectrum();
  show_cw_decode();
  show_menu();
  show_debug_value(debug_value);
  display_refresh();
}

static void init_menu(void)
{
  menu_window = 0;
  menu_current = 0;
  option_window = 0;
  option_current = 0;
  menu_button_state = MENU_WAIT_RELEASE_0;
  menu_button_action = MENU_NONE;
}

static const option_value_t process_menu(void)
{
  static uint32_t menu_timeout = millis()+MENU_TIMEOUT;
  option_value_t option = OPTION_NONE;
  switch (menu_button_state)
  {
    case MENU_WAIT_RELEASE_0:
    {
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        menu_button_state = MENU_BUTTON_IDLE;
        delay(50);
      }
      menu_timeout = millis()+MENU_TIMEOUT;
      break;
    }
    case MENU_BUTTON_IDLE:
    {
      // has the button been pressed?
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        menu_button_state = MENU_WAIT_RELEASE_1;
        menu_timeout = millis()+MENU_TIMEOUT;
        delay(50);
      }
      break;
    }
    case MENU_WAIT_RELEASE_1:
    {
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        menu_button_state = MENU_BUTTON_IDLE;
        switch (menu_button_action)
        {
          case MENU_NONE:          menu_button_action = MENU_TOP_SELECT;    break;
          case MENU_TOP_SELECT:    menu_button_action = MENU_OPTION_SELECT; break;
          case MENU_OPTION_SELECT: menu_button_action = MENU_NONE;          break;
        }
        delay(50);
      }
      menu_timeout = millis()+MENU_TIMEOUT;
      break;
    }
  }
  // process the button action
  switch (menu_button_action)
  {
    case MENU_NONE:
    {
      mutex_enter_blocking(&rotary_mutex);
      const int32_t rotary_delta = radio.tune;
      radio.tune = 0;
      mutex_exit(&rotary_mutex);
      if (rotary_delta>0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (menu_current<NUM_MENU_ITEMS-1)
        {
          menu_current++;
          if (menu_current>menu_window+3)
          {
            if (menu_window<NUM_MENU_ITEMS-1)
            {
              menu_window++;
            }
          }
        }
      }
      else if (rotary_delta<0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (menu_current>0)
        {
          menu_current--;
          if (menu_current<menu_window)
          {
            if (menu_window>0)
            {
              menu_window--;
            }          
          }
        }
      }
      break;
    }
    case MENU_TOP_SELECT:
    {
      if (menu_options[menu_current].menu_value==MENU_EXIT)
      {
        return OPTION_EXIT;
      }
      // process menu options
      const uint8_t num_options = menu_options[menu_current].num_options;
      mutex_enter_blocking(&rotary_mutex);
      const int32_t rotary_delta = radio.tune;
      radio.tune = 0;
      mutex_exit(&rotary_mutex);
      if (rotary_delta>0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (option_current<num_options-1)
        {
          option_current++;
          if (option_current>option_window+2)
          {
            if (option_window<num_options-1)
            {
              option_window++;
            }
          }
        }
      }
      else if (rotary_delta<0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (option_current>0)
        {
          option_current--;
          if (option_current<option_window)
          {
            if (option_window>0)
            {
              option_window--;
            }          
          }
        }
      }
      break;
    }
    case MENU_OPTION_SELECT:
    {
      option = menu_options[menu_current].options[option_current].option_value;
      radio.menu_active = false;
      break;
    }
  }
  if (millis()>menu_timeout)
  {
    radio.menu_active = false;
  }
  return option;
}

void __not_in_flash_func(loop)(void)
{
  // run DSP on core 0
  static bool tx = false;
  static uint32_t tx_peak_delay = 0;
  static float tx_level = 100.0f;
  if (tx)
  {
    // TX, check if changed to RX
    if (radio.tx_enable)
    {
      if (adc_value_ready)
      {
        volatile const uint32_t cpu_start = time_us_32();
        adc_value_ready = false;
        if ((radio.mode==MODE_DGL||radio.mode==MODE_DGU) && vox_mic_ready && abs(adc_value)>VOX_LEVEL)
        {
          vox_triggered = true;
        }
        int16_t tx_i = 0;
        int16_t tx_q = 0;
        switch (radio.mode)
        {
          case MODE_LSB: DSP::process_mic(adc_value,tx_q,tx_i,mic_gain,radio.micproc,radio.cessb); break;
          case MODE_USB: DSP::process_mic(adc_value,tx_i,tx_q,mic_gain,radio.micproc,radio.cessb); break;
          case MODE_DGL: DSP::process_dig(adc_value,tx_q,tx_i);                                    break;
          case MODE_DGU: DSP::process_dig(adc_value,tx_i,tx_q);                                    break;
          case MODE_CWL: CW::process_cw(radio.keydown,tx_i,tx_q);                                  break;
          case MODE_CWU: CW::process_cw(radio.keydown,tx_i,tx_q);                                  break;
          case MODE_FT8: tx_i = tx_q = FT8_TX_MAX;                                                 break;
        }
        tx_i = (int16_t)((float)tx_i * tx_level / 100.0f);
        tx_q = (int16_t)((float)tx_q * tx_level / 100.0f);
        tx_i = constrain(tx_i,-512,+511);
        tx_q = constrain(tx_q,-512,+511);
        dac_value_i_p = 512+tx_i;
        dac_value_i_n = 511-tx_i;
        dac_value_q_p = 512+tx_q;
        dac_value_q_n = 511-tx_q;
        adc_data_i[adc_sample_p] = tx_i<<5;
        adc_data_q[adc_sample_p] = tx_q<<5;
        adc_sample_p = (adc_sample_p+1) & (MAX_ADC_SAMPLES-1);
        if (radio.mode==MODE_LSB || radio.mode==MODE_USB ||
          radio.mode==MODE_DGL || radio.mode==MODE_DGU)
        {
          if (millis()>tx_peak_delay)
          {
            mic_peak_level = DSP::get_mic_peak_level(adc_value);
          }
          else
          {
            mic_peak_level = 0;
          }
        }
        else if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
        {
          // generate the sidetone
          int32_t dac_audio = CW::sidetone(radio.keydown);
          dac_audio = constrain(dac_audio,-2048l,+2047l);
          dac_audio += 2048l;
          dac_h = dac_audio >> 6;
          dac_l = dac_audio & 0x3f;
        }
        if (radio.mode!=MODE_FT8)
        {
          // only change TX level in TX mode
          static int32_t rotary = 0l;
          switch (r.process())
          {
            case DIR_CW:  rotary++; break;
            case DIR_CCW: rotary--; break;
          }
          if (rotary!=0)
          {
            // don't hang around if we can't own the mutex immediately
            // rotary will record any rotations
            if (mutex_try_enter(&rotary_mutex,0ul))
            {
              tx_level += (float)rotary;
              rotary = 0;
              mutex_exit(&rotary_mutex);
              tx_level = constrain(tx_level,0.0f,100.0f);
            }
          }
        }
        volatile const uint32_t cpu_end = time_us_32();
        cpu.usage += (cpu_end-cpu_start);
        cpu.count++;
      }
    }
    else
    {
      // switched to RX
      if (radio.mode==MODE_DGL || radio.mode==MODE_DGU)
      {
        enable_mic();
      }
      else
      {
        disable_mic();
      }
      // set TX output to zero in receive mode
      dac_value_i_p = 0;
      dac_value_i_n = 0;
      dac_value_q_p = 0;
      dac_value_q_n = 0;
      tx = false;
    }
  }
  else
  {
    // RX, check if changed to TX
    if (radio.tx_enable)
    {
      // switch to TX
      enable_mic();
      mic_gain = (float)radio.micgain / 100.0f;
      tx_peak_delay = millis() + 100u;
      tx = true;
    }
    else
    {
      if (adc_value_ready)
      {
        volatile const uint32_t cpu_start = time_us_32();
        adc_value_ready = false;
        if ((radio.mode==MODE_DGL||radio.mode==MODE_DGU) && vox_mic_ready && abs(adc_value)>VOX_LEVEL)
        {
          vox_triggered = true;
        }
        if (i2s.available())
        {
          int32_t ii = 0;
          int32_t qq = 0;
          i2s.read32(&ii, &qq);
          ii >>= 16;
          qq >>= 16;
          adc_data_i[adc_sample_p] = (int16_t)ii;
          adc_data_q[adc_sample_p] = (int16_t)qq;
          adc_sample_p = (adc_sample_p+1) & (MAX_ADC_SAMPLES-1);
          int32_t dac_audio = 0;
          float ft8_audio = 0.0f;
          float cwsig = 0.0f;
          const uint8_t jnr = radio.jnrlevel;
          const uint8_t nb = radio.nblevel;
          const uint8_t bw = radio.bandwidth - 1;
          switch (radio.mode)
          {
            case MODE_LSB: dac_audio = DSP::process_ssb(qq,ii,jnr,bw,nb); break;
            case MODE_USB: dac_audio = DSP::process_ssb(ii,qq,jnr,bw,nb); break;
            case MODE_AM:  dac_audio = DSP::process_am(ii,qq,jnr);        break;
            case MODE_CWL: dac_audio = DSP::process_cw(qq,ii,cwsig);      break;
            case MODE_CWU: dac_audio = DSP::process_cw(ii,qq,cwsig);      break;
            case MODE_DGL: dac_audio = DSP::process_dig(qq,ii);           break;
            case MODE_DGU: dac_audio = DSP::process_dig(ii,qq);           break;
            case MODE_FT8: ft8_audio = DSP::process_ft8(ii,qq);           break;
          }
          if (radio.mode==MODE_FT8)
          {
            dac_audio = 2048l;
            volatile static struct {uint32_t c : 2; } downsample = { 0 };
            if (downsample.c == 0)
            {
              // note, ft8_data is two buffers of FT8_FFT_SIZE/2 values
              ft8_data[ft8_sample_p] = ft8_audio;
              ft8_sample_p = (ft8_sample_p + 1) % FT8_FFT_SIZE;
            }
            downsample.c++;
          }
          else
          {
            dac_audio = constrain(dac_audio,-2048l,+2047l);
            dac_audio += 2048l;
          }
          dac_h = dac_audio >> 6;
          dac_l = dac_audio & 0x3f;
          if (radio.cwdecode!=0)
          {
            if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
            {
              static bool last_key_down = false;
              const uint8_t raw = radio.cwdecode==1?
                CWDECODE1::morse_decode(cwsig):
                CWDECODE2::morse_decode(cwsig);
              const bool cw_key_down = raw&0x80?true:false;
              const char raw_char = raw & 0x7f;
              if (cw_key_down != last_key_down || raw_char!='\0')
              {
                last_key_down = cw_key_down;
                mutex_enter_blocking(&cw_decode_mutex);
                cw_decode_buf[20] = '\0';
                cw_decode_buf[19] = cw_key_down?'*':' ';
                if (raw_char!='\0')
                {
                  for (uint8_t i=0;i<18;i++)
                  {
                    const char c = cw_decode_buf[i+1];
                    cw_decode_buf[i] = (c=='\0'?' ':c);
                  }
                  cw_decode_buf[18] = raw_char;
                }
                mutex_exit(&cw_decode_mutex);
              }
            }
            else
            {
              CWDECODE1::md1_initialised = 0;
              CWDECODE2::md2_initialised = 0;
              memset((char*)cw_decode_buf,0,sizeof(cw_decode_buf));
            }
          }
          else
          {
            CWDECODE1::md1_initialised = 0;
            CWDECODE2::md2_initialised = 0;
            memset((char*)cw_decode_buf,0,sizeof(cw_decode_buf));
          }
        }
        // only process tune in receive mode
        static int32_t rotary = 0l;
        switch (r.process())
        {
          case DIR_CW:  rotary++; break;
          case DIR_CCW: rotary--; break;
        }
        if (rotary!=0)
        {
          // don't hang around if we can't own the mutex immediately
          // rotary will record any rotations
          if (mutex_try_enter(&rotary_mutex,0ul))
          {
            radio.tune += rotary;
            rotary = 0;
            mutex_exit(&rotary_mutex);
          }
        }
        if (save_settings_now)
        {
          save_settings();
          save_settings_now = false;
        }
        volatile const uint32_t cpu_end = time_us_32();
        cpu.usage += (cpu_end-cpu_start);
        cpu.count++;
      }
    }
  }
}

static void process_spectrum(void)
{
  static uint32_t last_offset = UINT32_MAX;
  uint32_t offset = 0;
  const int8_t ga = (radio.mode==MODE_DGL || radio.mode==MODE_DGU)?-3:0;
  const int8_t gain = radio.tx_enable?ga:radio.level[radio.band];
  const uint32_t sample_p = adc_sample_p;
  if (sample_p<800)
  {
    // first half is in use, get the second half
    offset = SPECTRUM_BUFFER;
  }
  else if (sample_p>1023 && sample_p<1800)
  {
    // second half is in use, get the first half
    offset = 0;
  }
  else
  {
    return;
  }
  if (offset != last_offset)
  {
    last_offset = offset;
    int16_t data_re[SPECTRUM_BUFFER] = {0};
    int16_t data_im[SPECTRUM_BUFFER] = {0};
    for (uint32_t i=0;i<SPECTRUM_BUFFER;i++)
    {
      data_re[i] = adc_data_i[i+offset];
      data_im[i] = adc_data_q[i+offset];
    }
    spectrum::process(data_re,data_im,magnitude,gain);
  }
}


static void enable_ssb_tx(void)
{
  // 1. mute the receiver
  // 2. set TX/RX relay to TX
  // 3. enable MIC (DSP)
  // 4. enable TX bias

  // indicate PTT pressed
  digitalWrite(LED_BUILTIN,HIGH);

  // mute the receiver
  mute();
  delay(10);

  // disable QSD
  bpf_port.output(I2C_PIN_RXN,TCA9534::Level::H);
  delay(10);

  // enable MIC processing
  radio.tx_enable = true;
  set_frequency();
  update_display();
  delay(10);

  // enable QSE and TX bias
  lpf_port.output(I2C_PIN_TXENABLE,TCA9534::Level::H);
  delay(50);
  bpf_port.output(I2C_PIN_TXN,TCA9534::Level::L);
  digitalWrite(PIN_TXBIAS,HIGH);
}

static void ptt_release(void)
{
  // wait for PTT release
  uint32_t tx_display_update = 0;
  while (digitalRead(PIN_PTT)==LOW)
  {
    // spectrum data is mic input
    process_spectrum();

    // update display
    const uint32_t now = millis();
    if (now>tx_display_update)
    {
      update_display(mic_peak_level);
      tx_display_update = now + 50ul;
    }
  }
}

static void process_vox_tx(void)
{
  // wait for PTT release
  uint32_t tx_display_update = 0;

  // spectrum data is mic input
  process_spectrum();

  // update display
  const uint32_t now = millis();
  if (now>tx_display_update)
  {
    update_display(mic_peak_level);
    tx_display_update = now + 50ul;
  }
}

static void cw_dit_delay(const uint32_t ms,const uint32_t level)
{
  const uint32_t delay_time = millis()+ms;
  update_display(level);
  while (delay_time>millis())
  {
    // check for dah
    if (digitalRead(PIN_PADB)==LOW)
    {
      dah_latched = true;
    }
  }
}

static void cw_dah_delay(const uint32_t ms,const uint32_t level)
{
  const uint32_t delay_time = millis()+ms;
  update_display(level);
  while (delay_time>millis())
  {
    // check for dit
    if (digitalRead(PIN_PADA)==LOW)
    {
      dit_latched = true;
    }
  }
}

static void process_key(void)
{
  static const uint8_t spectrum_display_level = 16u;
  // disable QSD
  bpf_port.output(I2C_PIN_RXN,TCA9534::Level::H);

  // enable QSE and TX bias
  lpf_port.output(I2C_PIN_TXENABLE,TCA9534::Level::H);
  bpf_port.output(I2C_PIN_TXN,TCA9534::Level::L);
  digitalWrite(PIN_TXBIAS,HIGH);
  delay(10);

  // enable TX processing
  radio.keydown = false;
  radio.tx_enable = true;
  set_frequency();
  delay(10);

  // clear spectrum
  memset(magnitude,0,sizeof(magnitude));

  // stay here until timeout after key up (PTT released)
  uint32_t cw_timeout = millis() + CW_TIMEOUT;
  for (;;)
  {
    if (digitalRead(PIN_PTT)==LOW)
    {
      // indicate PTT pressed
      digitalWrite(LED_BUILTIN,HIGH);
      radio.keydown = true;
      magnitude[512] = spectrum_display_level;
      update_display(15u);
      while (digitalRead(PIN_PTT)==LOW)
      {
        update_display(15u);
      }
      // indicate PTT released
      digitalWrite(LED_BUILTIN,LOW);
      radio.keydown = false;
      magnitude[512] = 0u;
      update_display(0u);
      cw_timeout = millis() + CW_TIMEOUT;
    }
    if (digitalRead(PIN_PADA)==LOW || dit_latched)
    {
      // dit
      dit_latched = false;
      cw_dit_delay(radio.cw_dit,0u);
      radio.keydown = true;
      magnitude[512] = spectrum_display_level;
      digitalWrite(LED_BUILTIN,HIGH);
      cw_dit_delay(radio.cw_dit,15u);
      radio.keydown = false;
      magnitude[512] = 0u;
      digitalWrite(LED_BUILTIN,LOW);
      cw_timeout = millis() + CW_TIMEOUT;
    }
    if (digitalRead(PIN_PADB)==LOW || dah_latched)
    {
      // dah
      dah_latched = false;
      cw_dah_delay(radio.cw_dit,0u);
      radio.keydown = true;
      magnitude[512] = spectrum_display_level;
      digitalWrite(LED_BUILTIN,HIGH);
      cw_dah_delay(radio.cw_dit * 3, 15u);
      radio.keydown = false;
      magnitude[512] = 0u;
      digitalWrite(LED_BUILTIN,LOW);
      cw_timeout = millis() + CW_TIMEOUT;
    }
    if (millis()>cw_timeout)
    {
      break;
    }
  }

  // mute during transition back to receive
  mute();
}

static const radio_mode_t get_mode_auto(void)
{
  if (!radio.mode_auto)
  {
    return radio.mode;
  }
  if (radio.band==BAND_SWL)
  {
    return MODE_AM;
  }
  if (radio.frequency<10000000ul)
  {
    if (radio.frequency==7074000ul)
    {
      return MODE_USB;
    }
    if (radio.frequency==3573000ul)
    {
      return MODE_USB;
    }
    if (radio.frequency<7060000ul && radio.frequency>=7000000ul)
    {
      return MODE_CWL;
    }
    return MODE_LSB;
  }
  else if (radio.frequency<14060000ul && radio.frequency>=14000000ul)
  {
    return MODE_CWU;
  }
  return MODE_USB;
}

/*
 * FT8 processing
 */

//------------------------------------------------------------------------------
// FT8 STATIC DATA
//------------------------------------------------------------------------------
static monitor_t ft8mon = { 0 };

// Rolling history ring buffer - accumulates across slots, never cleared
static ft8_history_entry_t ft8_history[FT8_HISTORY_SIZE] = { 0 };
static uint32_t ft8_history_count = 0;
static uint32_t ft8_history_head = 0;

// Frozen display snapshot - taken when user first touches the rotary
static ft8_history_entry_t ft8_display_buf[FT8_HISTORY_SIZE] = { 0 };
static uint32_t ft8_display_count = 0;

// UI state
static bool ft8_new_available = false; // new decodes since last freeze
static ft8_ui_state_t ft8_ui_state = FT8_UI_AUTO;
static int32_t ft8_cursor = 0; // selected line in browse mode
static uint32_t ft8_last_interaction = 0; // millis() of last rotary/button

// Selected message for TX
static ft8_selected_t ft8_selected = { 0 };

// show QSO after completion
static ft8_qso_t ft8_qso = { 0 };
static uint32_t ft8_qso_display_until = 0;
static uint32_t ft8_exit_pending_until = 0;

// CQ state
static ft8_cq_t ft8_cq = { 0 };

//------------------------------------------------------------------------------
// DISPLAY FUNCTIONS
//------------------------------------------------------------------------------
static char ft8_sz_popup[64] = "";
static char ft8_sz_heading[64] = "";
static uint32_t ft8_popup_timeout = 0;

static void ft8_set_popup(const char *sz_message,const char *sz_heading = NULL)
{
  if (sz_message)
  {
    memset(ft8_sz_popup,0,sizeof(ft8_sz_popup));
    memset(ft8_sz_heading,0,sizeof(ft8_sz_heading));
    strncpy(ft8_sz_popup,sz_message,sizeof(ft8_sz_popup)-1);
    if (sz_heading)
    {
      strncpy(ft8_sz_heading,sz_heading,sizeof(ft8_sz_heading)-1);
    }
    ft8_popup_timeout = millis() + FT8_POPUP_TIME;
  }
}

static void ft8_show_popup(void)
{
  if (ft8_sz_popup[0] == '\0') return;
  if (millis() > ft8_popup_timeout)
  {
    ft8_sz_popup[0] = '\0';
    return;
  }

  // Filled box centred on screen
  lcd.fillRect(10, 48, 220, 40, LCD_BLUE);
  lcd.drawRect(10, 48, 220, 40, LCD_WHITE);

  lcd.setTextFont(1);
  lcd.setTextSize(1);
  lcd.setTextColor(LCD_YELLOW, LCD_BLUE);
  lcd.setCursor(16, 52);
  lcd.print(ft8_sz_heading[0]=='\0'?"QSO MESSAGE:":ft8_sz_heading);

  lcd.setTextSize(2);
  lcd.setTextColor(LCD_WHITE, LCD_BLUE);
  lcd.setCursor(16, 64);
  lcd.print(ft8_sz_popup);

  lcd.setTextFont(1);
  lcd.setTextSize(1);
}

static void ft8_show_frequency(void)
{
  const uint16_t tx_rx = radio.tx_enable?LCD_RED:LCD_WHITE;
  const uint16_t fcolour = radio.tx_safe?tx_rx:LCD_YELLOW;
  char sz_frequency[16] = "";
  memset(sz_frequency,0,sizeof(sz_frequency));
  ultoa(radio.frequency,sz_frequency,10);
  if (radio.frequency<1000000u)
  {
    // 6 digits
    //            012 345
    // 936000 ->  936.000
    // 012345    01234567
    sz_frequency[7] = sz_frequency[5];
    sz_frequency[6] = sz_frequency[4];
    sz_frequency[5] = sz_frequency[3];
    sz_frequency[4] = '.';
    sz_frequency[3] = sz_frequency[2];
    sz_frequency[2] = sz_frequency[1];
    sz_frequency[1] = sz_frequency[0];
    sz_frequency[0] = ' ';
  }
  else if (radio.frequency<10000000u)
  {
    // 7 digits
    // 3555000 -> 3.555.000
    // 0123456    012345678
    sz_frequency[9] = sz_frequency[6];
    sz_frequency[8] = sz_frequency[5];
    sz_frequency[7] = sz_frequency[4];
    sz_frequency[6] = '.';
    sz_frequency[5] = sz_frequency[3];
    sz_frequency[4] = sz_frequency[2];
    sz_frequency[3] = sz_frequency[1];
    sz_frequency[2] = '.';
    sz_frequency[1] = sz_frequency[0];
    sz_frequency[0] = ' ';
  }
  else
  {
    // 8 digits
    // 14222000 -> 14.222.000
    // 01234567    0123456789
    sz_frequency[9] = sz_frequency[7];
    sz_frequency[8] = sz_frequency[6];
    sz_frequency[7] = sz_frequency[5];
    sz_frequency[6] = '.';
    sz_frequency[5] = sz_frequency[4];
    sz_frequency[4] = sz_frequency[3];
    sz_frequency[3] = sz_frequency[2];
    sz_frequency[2] = '.';
  }
  lcd.setFreeFont(&Arial_Bold14pt7b);
  lcd.setTextSize(1);
  lcd.setTextColor(fcolour,LCD_BLACK);
  lcd.setCursor(FT8_FREQUENCY_X,FT8_FREQUENCY_Y);
  lcd.print(sz_frequency);
  lcd.setTextFont(1);
  lcd.setTextSize(1);
}

static void ft8_show_pulse(void)
{
  if (millis() % 1000 < 500)
  {
    lcd.setFreeFont(&Arial_Bold14pt7b);
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_BLUE,LCD_BLACK);
    lcd.setCursor(FT8_PULSE_X,FT8_PULSE_Y);
    lcd.print("*");
  }
  lcd.setTextFont(1);
  lcd.setTextSize(1);
}

static void ft8_show_progress(const uint32_t progress)
{
  if (progress == 0) return;
  const uint32_t bar = ((progress>240)?240:progress) - 1;
  lcd.drawLine(0,132,bar,132,radio.tx_enable?LCD_RED:LCD_GREEN);
  lcd.drawLine(0,133,bar,133,radio.tx_enable?LCD_RED:LCD_GREEN);
  lcd.drawLine(0,134,bar,134,radio.tx_enable?LCD_RED:LCD_GREEN);
}

// Show slot parity indicator and TX/RX status top-right
static void ft8_show_slot_status(const uint32_t slot_calibrate_ms,const ft8_state_t ft8_state)
{
  const uint32_t slot_number = (millis() - slot_calibrate_ms) / FT8_SLOT_MS;
  const bool even = (slot_number % 2) == 0;

  lcd.setTextFont(1);
  lcd.setTextSize(1);
  lcd.setCursor(FT8_STATUS_X, FT8_STATUS_Y);

  if (ft8_state == FT8_STATE_TRANSMITTING)
  {
    lcd.setTextColor(LCD_RED, LCD_BLACK);
    lcd.print("TX");
  }
  else
  {
    lcd.setTextColor(LCD_GREEN, LCD_BLACK);
    lcd.print("RX");
  }

  // show slot parity
  lcd.setTextColor(LCD_CYAN, LCD_BLACK);
  lcd.print(even ? " E" : " O");

  // flash CQ so it's obvious
  if (ft8_cq.active)
  {
    if (millis() % 1000 < 500) 
    {
      lcd.setCursor(FT8_CQ_X, FT8_CQ_Y);
      lcd.setTextColor(LCD_BLACK, LCD_YELLOW);
      lcd.print("CQ");
    }
  }
}

// Show browse/select position indicator "047/200"
static void ft8_show_position(void)
{
  lcd.setTextFont(1);
  lcd.setTextSize(1);
  lcd.setTextColor(LCD_CYAN, LCD_BLACK);
  lcd.setCursor(FT8_POSITION_X, FT8_POSITION_Y);
  char pos[16];
  snprintf(pos, sizeof(pos), "%3d/%3d", (int)(ft8_cursor + 1), (int)ft8_display_count);
  lcd.print(pos);
}

static void ft8_show_tx_frequency(void)
{
  if (ft8_cq.active)
  {
    lcd.setTextFont(1);
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_CYAN, LCD_BLACK);
    lcd.setCursor(FT8_TXAUDIO_X, FT8_TXAUDIO_Y);
    lcd.print((uint32_t)ft8_cq.audio_freq);
    return;
  }
  if (ft8_qso.active)
  {
    lcd.setTextFont(1);
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_CYAN, LCD_BLACK);
    lcd.setCursor(FT8_TXAUDIO_X, FT8_TXAUDIO_Y);
    lcd.print((uint32_t)ft8_qso.audio_freq);
    return;
  }
}

// Show [NEW] flash when new decodes arrived since last freeze
static void ft8_show_new_indicator(void)
{
  if (!ft8_new_available) return;
  if (millis() % 1000 >= 500) return;
  lcd.setTextFont(1);
  lcd.setTextSize(1);
  lcd.setCursor(FT8_NEW_X,FT8_NEW_Y);
  lcd.setTextColor(LCD_YELLOW, LCD_BLACK);
  lcd.print("NEW");
}

// Main decoded message display
// AUTO mode: live history, newest 12 entries, no cursor
// BROWSE/SELECTED: frozen snapshot, scrollable, cursor highlighted
static void ft8_show_decoded(const uint32_t slot_calibrate_ms)
{
  lcd.setTextFont(1);
  lcd.setTextSize(1);

  if (ft8_ui_state == FT8_UI_AUTO)
  {

    // Live view - show newest FT8_DISPLAY_LINES entries from history
    const int32_t total  = (int32_t)min(ft8_history_count, (uint32_t)FT8_HISTORY_SIZE);
    if (total == 0)
    {
      lcd.setCursor(0, 40);
      lcd.setTextColor(LCD_DARKGREY, LCD_BLACK);
      lcd.print("Waiting for FT8 signals...");
      return;
    }
    const int32_t lines_show = (total < (int32_t)FT8_DISPLAY_LINES) ? total : (int32_t)FT8_DISPLAY_LINES;
    const int32_t from = total - lines_show;   // always >= 0, no underflow possible
    lcd.setCursor(0, 30);
    for (int32_t i = from; i < total; i++)
    {
      const ft8_history_entry_t* e = ft8_history_get(i);
      lcd.setTextColor(LCD_WHITE, LCD_BLACK);
      char buf[FT8_DISPLAY_COLS + 1];
      strncpy(buf, e->text, FT8_DISPLAY_COLS);
      buf[FT8_DISPLAY_COLS] = '\0';
      if (strstr(buf,YOUR_CALL) !=NULL)
      {
        lcd.setTextColor(LCD_WHITE, LCD_RED);
      }
      else if (strstr(buf,"CQ ") !=NULL)
      {
        lcd.setTextColor(LCD_WHITE, LCD_BLUE);
      }
      lcd.println(buf);
    }
  }
  else
  {
    // Browse/selected mode - frozen snapshot with scrolling cursor
    // Keep cursor visible: window follows cursor
    const int32_t total = (int32_t)ft8_display_count;
    int32_t from = ft8_cursor - FT8_DISPLAY_LINES + 3; // cursor near bottom
    if (from < 0) from = 0;
    if (from > total - FT8_DISPLAY_LINES) from = max(0, total - FT8_DISPLAY_LINES);

    // Age warning threshold - entries older than this many slots
    const uint8_t current_slot_low = (uint8_t)((millis() - slot_calibrate_ms) / FT8_SLOT_MS);

    lcd.setCursor(0, 30);
    for (int32_t i = from; i < from + FT8_DISPLAY_LINES && i < total; i++)
    {
      const ft8_history_entry_t* e = &ft8_display_buf[i];
      const bool is_cursor   = (i == ft8_cursor);
      const bool is_selected = (ft8_ui_state == FT8_UI_SELECTED) && is_cursor;

      // Message text (truncated)
      char buf[FT8_DISPLAY_COLS - 1];
      memset(buf,0,sizeof(buf));
      strncpy(buf, e->text, sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';

      // Age check: warn if entry is more than 4 slots old
      const uint8_t age = current_slot_low - e->slot_number_low;
      const bool is_old = (age > 4);
      const bool is_cq = strstr(buf,"CQ ") != NULL;
      const bool is_mycall = strstr(buf,YOUR_CALL) != NULL;

      // Slot parity indicator prefix (1 char)
      if (is_selected) lcd.setTextColor(LCD_BLACK, LCD_GREEN);
      else if (is_cursor) lcd.setTextColor(LCD_BLACK, LCD_WHITE);
      else if (is_mycall) lcd.setTextColor(LCD_WHITE, LCD_RED);
      else if (is_cq) lcd.setTextColor(LCD_WHITE, LCD_BLUE);
      else if (is_old) lcd.setTextColor(LCD_DARKGREY, LCD_BLACK);
      else lcd.setTextColor(LCD_WHITE, LCD_BLACK);

      // Parity prefix: E=even slot, O=odd slot
      lcd.print(e->received_in_even_slot ? "E" : "O");
      lcd.print(" ");
      lcd.println(buf);
    }
    lcd.setTextColor(LCD_WHITE, LCD_BLACK); // restore
    ft8_show_position();
    ft8_show_new_indicator();
  }
}

//------------------------------------------------------------------------------
// ft8_qso_display()
// Replaces ft8_show_decoded() during an active QSO.
// Shows all 6 exchange rows, each colour-coded by state.
//------------------------------------------------------------------------------
static void ft8_qso_display(void)
{
  lcd.setTextFont(1);
  lcd.setTextSize(2);
  lcd.setCursor(0, 30);

  bool any_timeout = false;
  for (int i = 0; i < ft8_qso.num_rows; i++)
  {
    const ft8_qso_row_t* row = &ft8_qso.rows[i];
    uint16_t fg = 0;
    uint16_t bg = 0;
    switch (row->state)
    {
      case FT8_QSO_ROW_PENDING: fg = LCD_DARKGREY; bg = LCD_BLACK; break;
      case FT8_QSO_ROW_CURRENT: fg = LCD_BLACK;    bg = LCD_WHITE; break;
      case FT8_QSO_ROW_WAITING: fg = LCD_YELLOW;   bg = LCD_BLACK; break;
      case FT8_QSO_ROW_RETRY:   fg = LCD_PINK;     bg = LCD_BLACK; break;
      case FT8_QSO_ROW_TIMEOUT: fg = LCD_RED;      bg = LCD_BLACK; break;
      case FT8_QSO_ROW_DONE:    fg = LCD_GREEN;    bg = LCD_BLACK; break;
      default:                  fg = LCD_WHITE;    bg = LCD_BLACK; break;
    }
    if (row->state == FT8_QSO_ROW_TIMEOUT) any_timeout = true;
    lcd.setTextColor(fg, bg);

    // Row 0 is the heard CQ (no TX/RX prefix)
    // Rows 1-5 show TX/RX
    if (i > 0)
      lcd.print(row->is_tx ? "T " : "R ");
    else
      lcd.print("  ");
    char buf[FT8_DISPLAY_COLS - 1];
    memset(buf,0,sizeof(buf));
    strncpy(buf, row->text, sizeof(buf) - 1);
    buf[18] = '\0';
    lcd.println(buf);
  }

  lcd.setTextColor(LCD_WHITE, LCD_BLACK);

  // only fire once
  if (any_timeout && !ft8_qso.popup_shown)
  {
    ft8_qso.popup_shown = true;
    ft8_set_popup("Hold to abort");
  }
}

static void ft8_show_swr(void)
{
  static const uint32_t PO_DECAY_RATE = 250ul;
  static const float A = 0.141570128f;
  static const float B = 5.64859373f;
  volatile static uint32_t max_po = 0; 
  volatile static uint32_t po_decay = 0;
  if (!radio.tx_enable)
  {
    max_po = 0;
    return;
  }
  const uint32_t adc_fwd_raw = (uint32_t)fwdADC.read();
  const uint32_t adc_ref_raw = (uint32_t)refADC.read();
  if (adc_fwd_raw==0xffffu || adc_ref_raw==0xffffu)
  {
    return;
  }

  // vswr
  uint32_t vswr = 100u;
  const float adcfwd = (float)(FILTER::ma1(adc_fwd_raw));
  const float adcref = (float)(FILTER::ma2(adc_ref_raw));
  const float vfwd = A * adcfwd + B;
  const float vref = A * adcref + (adcref>2.0f?B:0.0f);
  if (vfwd>vref)
  {
    vswr = (uint32_t)(100.0f * (vfwd + vref) / (vfwd - vref));
    vswr = constrain(vswr,100u,999u);
  }
  // power out
  // Vpp * Vpp / 400
  const uint32_t now = millis();
  const float fwd = A * (float)adc_fwd_raw + (adc_fwd_raw>2?B:0.0f);
  const uint32_t po = (uint32_t)(((fwd * fwd) / 400.0f) * 10.0f + 0.5f);
  if (po<max_po)
  {
    if (now>po_decay)
    {
      if (max_po>0) max_po--;
      po_decay = now + PO_DECAY_RATE;
    }
  }
  else
  {
    max_po = po;
    po_decay = now + PO_DECAY_RATE;
  }

  // display SWR
  char sz_swr[16] = "";
  memset(sz_swr,0,sizeof(sz_swr));
  ultoa(vswr,sz_swr,10);
  sz_swr[3] = sz_swr[2];
  sz_swr[2] = sz_swr[1];
  sz_swr[1] = '.';
  lcd.setTextFont(1);
  lcd.setTextSize(1);
  lcd.setTextColor(LCD_WHITE);
  lcd.setCursor(FT8_SWR_X,FT8_SWR_Y);
  lcd.print(sz_swr);

  // display power
  char sz_po[16] = "";
  memset(sz_po,0,sizeof(sz_po));
  ultoa(max_po,sz_po,10);
  if (max_po<10)
  {
    sz_po[2] = sz_po[0];
    sz_po[1] = '.';
    sz_po[0] = '0';
  }
  else if (max_po<100)
  {
    sz_po[2] = sz_po[1];
    sz_po[1] = '.';
  }
  else
  {
    sz_po[3] = sz_po[2];
    sz_po[2] = '.';
  }
  lcd.setCursor(FT8_PWR_X,FT8_PWR_Y);
  lcd.print(sz_po);
}

static void ft8_show_calibrating(void)
{
  const uint32_t now = millis() % 1000000;
  lcd.setTextFont(1);
  lcd.setTextSize(2);
  lcd.setTextColor(LCD_RED);
  lcd.setCursor(0,30);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
  lcd.setTextColor(LCD_WHITE);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
  lcd.setTextColor(LCD_GREEN);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
  lcd.setTextColor(LCD_BLUE);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
  lcd.setTextColor(LCD_PURPLE);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
  lcd.setTextColor(LCD_YELLOW);
  lcd.print("Calibrate:");
  lcd.print(now);
  lcd.println("ms");
}

static void ft8_display(
  const uint32_t slot_calibrate_ms,
  const ft8_state_t ft8_state,
  const uint32_t progress = 0,
  const bool calibrating = false)
{
  display_clear();
  ft8_show_pulse();
  ft8_show_swr();
  ft8_show_frequency();
  ft8_show_progress(progress);
  ft8_show_slot_status(slot_calibrate_ms, ft8_state);
  ft8_show_tx_frequency();
  if (calibrating)
  {
    ft8_show_calibrating();
    display_refresh();
    return;
  }
  if (ft8_qso.active || millis() < ft8_qso_display_until)
    ft8_qso_display();
  else
    ft8_show_decoded(slot_calibrate_ms);
  ft8_show_popup();
  display_refresh();
}

//------------------------------------------------------------------------------
// HISTORY RING BUFFER
//------------------------------------------------------------------------------

// Get the i-th entry in chronological order (0=oldest, count-1=newest)
static const ft8_history_entry_t* ft8_history_get(uint32_t i)
{
  // if buffer not full yet
  uint32_t idx = i;
  if (ft8_history_count >= FT8_HISTORY_SIZE)
    // oldest is at head
    idx = (ft8_history_head + i) % FT8_HISTORY_SIZE;
  return &ft8_history[idx];
}

// Append decoded messages from one slot into the rolling history
static void ft8_history_add(
  const char lines[][FTX_MAX_DISPLAY_LENGTH],
  const uint32_t count,
  const bool even_slot,
  const uint8_t slot_num)
{
  if (count==0) return;
  for (uint32_t i = 0; i < count; i++)
  {
    ft8_history_entry_t* entry = &ft8_history[ft8_history_head];
    memcpy(entry->text, lines[i], FTX_MAX_DISPLAY_LENGTH);
    entry->text[FTX_MAX_DISPLAY_LENGTH - 1] = '\0';
    entry->received_in_even_slot = even_slot;
    entry->slot_number_low = slot_num;
    ft8_history_head = (ft8_history_head + 1) % FT8_HISTORY_SIZE;
    if (ft8_history_count < FT8_HISTORY_SIZE) ft8_history_count++;
  }
  ft8_new_available = true;
}

// Snapshot live history into the frozen display buffer
// Called once when user first moves the rotary encoder
static void ft8_freeze_display(void)
{
  ft8_display_count = (ft8_history_count < FT8_HISTORY_SIZE)? ft8_history_count : FT8_HISTORY_SIZE;
  for (uint32_t i = 0; i < ft8_display_count; i++)
    ft8_display_buf[i] = *ft8_history_get(i);
  // start at most recent
  ft8_cursor = (int32_t)ft8_display_count - 1;
  ft8_new_available = false;
}

//------------------------------------------------------------------------------
// ROTARY ENCODER HANDLER
// Call this every tick - reads radio.tune via rotary_mutex (same pattern as
// the rest of the codebase)
//------------------------------------------------------------------------------
static void ft8_handle_rotary(void)
{
  // Read and consume rotary delta
  if (!mutex_try_enter(&rotary_mutex,0ul))
  {
    // do not block but try again later
    return;
  }
  const int32_t delta = radio.tune;
  radio.tune = 0;
  mutex_exit(&rotary_mutex);

  if (delta == 0) return;

  ft8_last_interaction = millis();
  ft8_exit_pending_until = 0;
  switch (ft8_ui_state)
  {
    case FT8_UI_AUTO:
    {
      // First rotary movement - freeze display and enter browse mode
      if (ft8_history_count > 0)
      {
        ft8_freeze_display();
        ft8_ui_state = FT8_UI_BROWSE;
      }
      break;
    }
    case FT8_UI_BROWSE:
    {
      // Scroll cursor through frozen display buffer
      // CCW (delta < 0) moves toward older entries (lower index)
      // CW  (delta > 0) moves toward newer entries (higher index)
      ft8_cursor += delta;
      if (ft8_cursor < 0) ft8_cursor = 0;
      if (ft8_cursor >= (int32_t)ft8_display_count) ft8_cursor = (int32_t)ft8_display_count - 1;
      break;
    }
    case FT8_UI_SELECTED:
    {
      break;
    }
  }
}

//------------------------------------------------------------------------------
// DECODE CALLBACK
// Called every N candidates during ft8_decode() so the UI stays responsive.
//------------------------------------------------------------------------------
static uint32_t ft8_cal_freeze = 0;
static void ft8_ui_callback(void)
{
  // just update display and monitor rotary
  // but only in browse mode
  if (ft8_ui_state != FT8_UI_BROWSE) return;
  ft8_handle_rotary();

  // update display but only every 50ms
  static uint32_t next_update = 0;
  if (millis() > next_update)
  {
    next_update = millis() + 50ul;
    ft8_display(ft8_cal_freeze, FT8_STATE_DECODING);
  }
}

//------------------------------------------------------------------------------
// ft8_cq_start()
// Find a free frequency and arm CQ TX for the next slot.
// Sets ft8_cq.active = true on success.
//------------------------------------------------------------------------------
static bool ft8_cq_start(const uint32_t slot_calibrate_ms)
{
  if (ft8_cq.active) return false;   // already running
  if (ft8_qso.active) return false;   // QSO has priority

  const float freq = ft8_find_free_freq();
  if (freq <= 0.0f)
  {
    ft8_set_popup("No free freq","INFO:");
    return false;
  }

  // TX in the very next slot (opposite parity to current)
  const uint32_t now = millis();
  const uint32_t slot_num = (now - slot_calibrate_ms) / FT8_SLOT_MS;
  const bool this_even = (slot_num % 2) == 0;

  ft8_cq.audio_freq = freq;
  ft8_cq.even_slot = !this_even;
  ft8_cq.attempts = 0;
  ft8_cq.active = true;

  LOG(LOG_INFO, "CQ start: %.0f Hz, next %s slot\n",
    freq, !this_even ? "EVEN" : "ODD");
  return true;
}

//------------------------------------------------------------------------------
// ft8_cq_transmit()
// Builds and sends one CQ message. Increments attempts counter.
//------------------------------------------------------------------------------
static bool ft8_cq_transmit(const uint32_t slot_calibrate_ms, const ft8_state_t ft8_state)
{
  char msg[FTX_MAX_MESSAGE_LENGTH];
  snprintf(msg, sizeof(msg), "CQ %s %s", YOUR_CALL, YOUR_GRID);

  uint8_t tones[FT8_NN];
  memset(tones, 0, sizeof(tones));
  if (!ft8_encode_message(msg, tones))
  {
    LOG(LOG_ERROR, "CQ encode failed\n");
    return false;
  }

  const float tone0 = (float)tones[0] * FT8_TONE_SPACING;
  set_frequency((uint32_t)((ft8_cq.audio_freq + tone0) * 100.0f));

  ft8_enable_tx();
  const bool ok = ft8_transmit_tones(tones, ft8_cq.audio_freq,
    slot_calibrate_ms, ft8_state);
  ft8_disable_tx();

  ft8_cq.attempts++;
  LOG(LOG_INFO, "CQ sent (attempt %d) at %.0f Hz\n",
    ft8_cq.attempts, ft8_cq.audio_freq);
  return ok;
}

//------------------------------------------------------------------------------
// PARSE A CQ DISPLAY LINE
//
// Input format (from ft8_show_decoded snprintf):
//   "+02.5 +0.12  998 ~  CQ VK3ABC QF22"
//    snr   dt    freq     message
//
// Extracts:
//   audio_freq  - the audio offset frequency in Hz (e.g. 998)
//   their_call  - the calling station's callsign (e.g. "VK3ABC")
//
// Returns true if a valid CQ was parsed.
//------------------------------------------------------------------------------
static bool ft8_parse_cq(
  const char* display_text,
  float&      audio_freq_out,
  char*       their_call_out, // buffer must be >= 12 chars
  char*       their_grid_out) // buffer must be >= 8 chars, may be empty
{
  if (!display_text || !their_call_out) return false;

  their_call_out[0] = '\0';
  if (their_grid_out) their_grid_out[0] = '\0';

  // Parse the three numeric fields and the message portion
  float snr = 0.0f;
  float dt = 0.0f;
  float freq = 0.0f;
  char msg[FTX_MAX_MESSAGE_LENGTH] = "";

  // Format: "%+05.1f %+4.2f %4.0f ~  %s"
  // Use %*c to skip the "~" and surrounding spaces
  const int n = sscanf(display_text,"%f %f %f %34[^\n]",&snr, &dt, &freq, msg);
  if (n < 4 || freq <= 0.0f) return false;

  audio_freq_out = freq;

  // msg should now be e.g. "CQ VK3ABC QF22" or "CQ DX VK3ABC QF22"
  // Ensure it starts with "CQ" (case insensitive for safety)
  if (strncmp(msg, "CQ", 2) != 0 && strncmp(msg, "cq", 2) != 0) return false;

  // Tokenise: skip "CQ", optionally skip "DX"/"nnn"/"abcd" modifier
  char tokens[4][16] = { "", "", "", "" };
  const char* p = msg;
  p = copy_token(tokens[0], sizeof(tokens[0]), p); // "CQ"
  p = copy_token(tokens[1], sizeof(tokens[1]), p); // callsign OR modifier
  p = copy_token(tokens[2], sizeof(tokens[2]), p); // grid OR callsign
  p = copy_token(tokens[3], sizeof(tokens[3]), p); // grid (if modifier present)

  // Determine which token is the callsign.
  // A standard callsign contains at least one digit and is 3-8 chars.
  // A grid square is exactly 4 chars: 2 letters + 2 digits.
  // A modifier is "DX" or regional code or 3-digit number or 4 letters.
  const char* callsign = NULL;
  const char* grid = NULL;

  if (tokens[2][0] != '\0')
  {
    // Check if tokens[1] looks like a modifier (DX, or 3 digits, or 4 letters)
    bool t1_is_modifier = false;
    if (equals(tokens[1], "DX"))
    {
      t1_is_modifier = true;
    }
    else
    {
      // 2-letter alpha = regional code (DX, NA, EU, SA, AS, AF, OC, plus future)
      const int t1_len = (int)strlen(tokens[1]);
      bool all_alpha = true;
      bool all_digit = true;
      for (int i = 0; tokens[1][i]; i++)
      {
        if (!is_letter(tokens[1][i])) all_alpha = false;
        if (!is_digit(tokens[1][i])) all_digit = false;
      }
      if ((all_alpha && t1_len == 2) || // regional codes
          (all_alpha && t1_len == 4) || // 4-letter named modifiers
          (all_digit && t1_len == 3))   // 3-digit number
        t1_is_modifier = true;
    }
    callsign = t1_is_modifier ? tokens[2] : tokens[1];
    grid = t1_is_modifier ? tokens[3] : tokens[2];
  }
  else if (tokens[1][0] != '\0')
  {
    callsign = tokens[1];  // just "CQ VK3ABC" with no grid
  }

  if (!callsign || callsign[0] == '\0') return false;

  strncpy(their_call_out, callsign, 11);
  their_call_out[11] = '\0';

  if (their_grid_out && grid && grid[0] != '\0')
  {
    strncpy(their_grid_out, grid, 7);
    their_grid_out[7] = '\0';
  }

  return true;
}

static bool ft8_parse_direct_call(
  const char* display_text,
  float& audio_freq_out,
  char* their_call_out,   // >= 12 chars
  char* their_extra_out)  // >= 8 chars: grid OR report, may be empty
{
  if (!display_text || !their_call_out) return false;

  their_call_out[0] = '\0';
  if (their_extra_out) their_extra_out[0] = '\0';

  float snr = 0.0f;
  float dt = 0.0f;
  float freq = 0.0f;
  char msg[FTX_MAX_MESSAGE_LENGTH] = "";
  if (sscanf(display_text, "%f %f %f %34[^\n]", &snr, &dt, &freq, msg) < 4)
    return false;
  if (freq <= 0.0f) return false;

  audio_freq_out = freq;

  // Message must be "OURCALL THEIRCALL EXTRA"
  char tokens[3][16] = { "", "", "" };
  const char* p = msg;
  p = copy_token(tokens[0], sizeof(tokens[0]), p);  // should be YOUR_CALL
  p = copy_token(tokens[1], sizeof(tokens[1]), p);  // their callsign
  p = copy_token(tokens[2], sizeof(tokens[2]), p);  // grid or report

  if (!equals(tokens[0], YOUR_CALL)) return false;
  if (tokens[1][0] == '\0')          return false;

  strncpy(their_call_out, tokens[1], 11);
  their_call_out[11] = '\0';

  if (their_extra_out && tokens[2][0] != '\0')
  {
    strncpy(their_extra_out, tokens[2], 7);
    their_extra_out[7] = '\0';
  }

  return true;
}

//------------------------------------------------------------------------------
// ENABLE TX  (simplified enable_ssb_tx - no update_display, no mic)
//------------------------------------------------------------------------------
static void ft8_enable_tx(void)
{
  digitalWrite(LED_BUILTIN, HIGH);

  // Mute and disable receiver
  mute();
  delay(10);
  bpf_port.output(I2C_PIN_RXN, TCA9534::Level::H);
  delay(10);

  // Set TX mode (no mic/DSP enable needed for FT8 - Si5351 drives QSE directly)
  radio.tx_enable = true;
  // Note: set_frequency() called per-tone in the TX loop

  delay(10);

  // Enable QSE and TX bias
  lpf_port.output(I2C_PIN_TXENABLE, TCA9534::Level::H);
  delay(50);
  bpf_port.output(I2C_PIN_TXN, TCA9534::Level::L);
  digitalWrite(PIN_TXBIAS, HIGH);
}

//------------------------------------------------------------------------------
// DISABLE TX  - return to receive
//------------------------------------------------------------------------------
static void ft8_disable_tx(void)
{
  radio.tx_enable = false;
  digitalWrite(PIN_TXBIAS, LOW);
  bpf_port.output(I2C_PIN_TXN, TCA9534::Level::H);
  bpf_port.output(I2C_PIN_RXN, TCA9534::Level::L);
  lpf_port.output(I2C_PIN_TXENABLE, TCA9534::Level::L);
  digitalWrite(LED_BUILTIN, LOW);

  // Restore RX frequency (no ft8_offset)
  set_frequency();
  unmute();
}

//------------------------------------------------------------------------------
// TRANSMIT FT8 TONES
//
// Transmits FT8_NN (79) tones, each FT8_SYMBOL_MS (160ms).
// audio_base_hz: the audio frequency offset in Hz (e.g. 998)
// tones[]:       tone indices 0..7 from ft8_encode_message()
//
// ft8_offset passed to set_frequency() = audio_base_hz + tone * 6.25 Hz
// Since set_frequency takes uint32_t, we round to nearest Hz.
// 6.25 Hz resolution: error < 0.5 Hz per tone, well within FT8 tolerance.
//
// Display is updated every 8 tones (~1.3s) so progress is visible.
//------------------------------------------------------------------------------
static const bool ft8_transmit_tones(
  const uint8_t* tones,
  const float audio_base_hz,
  const uint32_t slot_calibrate_ms,
  const ft8_state_t ft8_state)
{
  uint32_t first_press = 0;
  absolute_time_t tone_process_next = make_timeout_time_ms(FT8_SYMBOL_MS);
  for (int i = 0; i < FT8_NN; i++)
  {
    // Calculate SI5351 offset for this tone:
    //  offset = (audio_base_hz + tones[i] * 6.25 Hz) * 100
    //set_frequency takes integer Hz (* 100)
    const float tone = (float)tones[i] * FT8_TONE_SPACING;
    const uint32_t offset_100hz = (uint32_t)((audio_base_hz + tone) * 100.0f);
    set_frequency(offset_100hz);

    const uint32_t progress = UTIL::map(i+1, 0, FT8_NN, 0, 240);
    ft8_display(slot_calibrate_ms, ft8_state, progress);

    // if long press then exit
    if (first_press == 0)
    {
      if (digitalRead(PIN_ENCBUT) == LOW)
      {
        first_press = millis();
      }
    }
    else
    {
      if (digitalRead(PIN_ENCBUT) == HIGH)
      {
        if (millis() - first_press > FT8_BUTTON_LONG_PRESS)
        {
          return false;
        }
        first_press = 0;
      }
    }

    busy_wait_until(tone_process_next);
    tone_process_next = delayed_by_ms(tone_process_next,FT8_SYMBOL_MS);
  }
  return true;
}

//------------------------------------------------------------------------------
// Find a stretch of frequency that's been quiet over the last several FT8
// slots, wide enough to safely transmit a CQ without interfering with
// existing signals. The function does this by looking at the waterfall
// (the recent history of FFT magnitudes) and finding the longest run
// of consecutive bins whose energy has stayed low.
//------------------------------------------------------------------------------
static const float ft8_find_free_freq(void)
{
  const ftx_waterfall_t* wf = &ft8mon.wf;
  if (wf->num_blocks < 4) return 0.0f;

  // Convert CQ TX range to FFT bin indices (one bin = 3.125 Hz)
  const int fft_min = (int)(FT8_CQ_TX_MIN_HZ / 3.125f);     // 96
  const int fft_max = (int)(FT8_CQ_TX_MAX_HZ / 3.125f);     // 800

  // Convert to waterfall storage offsets
  // Waterfall slot for fft_bin = (fft_bin / FREQ_OSR - FT8_MIN_BIN, fft_bin % FREQ_OSR)
  const int scan_blocks = (wf->num_blocks < 4) ? wf->num_blocks : 4;
  const int start_block = wf->num_blocks - scan_blocks;

  // Peak magnitude per FFT bin across recent blocks (one entry per 3.125 Hz)
  const int scan_len = fft_max - fft_min;
  uint8_t peak[800] = { 0 };   // size to fit any range

  for (int b = start_block; b < wf->num_blocks; b++)
  {
    const uint8_t* block = wf->mag + b * FT8_BLOCK_STRIDE;
    // Use time_sub=0 only for the scan (one full row per freq_sub)
    for (int fs = 0; fs < FT8_FREQ_OSR; fs++)
    {
      const uint8_t* row = block + fs * FT8_WF_NUM_BINS;
      for (int fft_bin = fft_min; fft_bin < fft_max; fft_bin++)
      {
        const int wf_bin = (fft_bin / FT8_FREQ_OSR) - FT8_MIN_BIN;
        const int wf_fs  = fft_bin % FT8_FREQ_OSR;
        if (wf_fs != fs) continue;
        if (wf_bin < 0) continue;
        if (wf_bin >= FT8_WF_NUM_BINS) continue;
        const uint8_t v = row[wf_bin];
        const int idx = fft_bin - fft_min;
        if (v > peak[idx]) peak[idx] = v;
      }
    }
  }

  // Slot-wide noise floor (25th percentile) — stable across noise variance
  const int nf = ft8_estimate_noise_floor(wf);
  // 15 dB above noise floor
  const uint8_t QUIET = (uint8_t)(nf + 30);
  const int NEEDED = 24;

  int best_start = -1, best_len = 0;
  int cur_start  =  0, cur_len  = 0;

  for (int i = 0; i < scan_len; i++)
  {
    if (peak[i] < QUIET)
    {
      if (cur_len == 0) cur_start = i;
      cur_len++;
      if (cur_len > best_len) { best_len = cur_len; best_start = cur_start; }
    }
    else cur_len = 0;
  }

  if (best_len < NEEDED) return 0.0f;

  // Centre of the quietest run, converted back to Hz
  const int centre_fft_bin = fft_min + best_start + best_len / 2;
  const float freq_hz = (float)centre_fft_bin * 3.125f;

  LOG(LOG_INFO, "Free freq: %.0f Hz (run %d bins = %.0f Hz)\n",
      freq_hz, best_len, best_len * 3.125f);

  return freq_hz;
}
//------------------------------------------------------------------------------
// ft8_qso_start()
// Called when user confirms a CQ selection (button press in BROWSE mode).
// Parses the CQ display line, builds all 6 QSO rows.
// Sets ft8_qso.active = true on success.
// Non-CQ selections leave ft8_qso.active = false (single TX, no QSO tracking).
//------------------------------------------------------------------------------
static void ft8_qso_start(void)
{
  memset(&ft8_qso, 0, sizeof(ft8_qso));

  float snr_f = 0.0f, dt = 0.0f, freq = 0.0f;
  char  msg[FTX_MAX_MESSAGE_LENGTH] = "";
  if (sscanf(ft8_selected.text, "%f %f %f %34[^\n]",
    &snr_f, &dt, &freq, msg) < 4) return;
  if (freq <= 0.0f) return;

  char their_call[12] = "";
  char their_extra[8] = "";   // grid (from CQ) or report (from direct call)
  bool is_cq  = false;
  bool is_direct = false;

  // Try CQ first, then direct call
  if (ft8_parse_cq(ft8_selected.text, freq, their_call, their_extra))
    is_cq = true;
  else if (ft8_parse_direct_call(ft8_selected.text, freq, their_call, their_extra))
    is_direct = true;
  else
    // neither — leave ft8_qso.active = false
    return;

  ft8_qso.audio_freq = freq;
  strncpy(ft8_qso.their_call, their_call, sizeof(ft8_qso.their_call) - 1);

  // Row 0: the heard message
  strncpy(ft8_qso.rows[0].text, msg, FTX_MAX_DISPLAY_LENGTH - 1);
  ft8_qso.rows[0].is_tx = false;
  ft8_qso.rows[0].state = FT8_QSO_ROW_DONE;

  if (is_direct)
  {
    // R1 (row 0 - heard): VK7IAN W1XYZ KO45 - already set above

    // T2 (row 1): W1XYZ VK7IAN -07 (our signal report for them)
    snprintf(ft8_qso.rows[1].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s %s", their_call, YOUR_CALL, FT8_DEFAULT_REPORT);
    ft8_qso.rows[1].is_tx = true;
    ft8_qso.rows[1].state = FT8_QSO_ROW_CURRENT;

    // R3 (row 2): VK7IAN W1XYZ R-09 or RR73
    snprintf(ft8_qso.rows[2].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s R???", YOUR_CALL, their_call);
    ft8_qso.rows[2].is_tx = false;
    ft8_qso.rows[2].state = FT8_QSO_ROW_PENDING;

    // T4 (row 3): W1XYZ VK7IAN RRR (updated when R3 known)
    snprintf(ft8_qso.rows[3].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s RRR", their_call, YOUR_CALL);
    ft8_qso.rows[3].is_tx = true;
    ft8_qso.rows[3].state = FT8_QSO_ROW_PENDING;

    // R5 (row 4): VK7IAN W1XYZ 73  (exit anyway on timeout)
    snprintf(ft8_qso.rows[4].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s 73", YOUR_CALL, their_call);
    ft8_qso.rows[4].is_tx = false;
    ft8_qso.rows[4].state = FT8_QSO_ROW_PENDING;

    ft8_qso.num_rows = 5;   // rows 0-4
  }
  else
  {
    // ── Responding to CQ ──────────────────────────────────────────────
    // Row 1 TX: THEIRCALL OURCALL OURGRID
    snprintf(ft8_qso.rows[1].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s %s", their_call, YOUR_CALL, YOUR_GRID);
    ft8_qso.rows[1].is_tx = true;
    ft8_qso.rows[1].state = FT8_QSO_ROW_CURRENT;

    // Row 2 RX: they send us a signal report
    snprintf(ft8_qso.rows[2].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s ???", YOUR_CALL, their_call);
    ft8_qso.rows[2].is_tx = false;
    ft8_qso.rows[2].state = FT8_QSO_ROW_PENDING;

    // Row 3 TX: THEIRCALL OURCALL R+report (filled when step 2 known)
    snprintf(ft8_qso.rows[3].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s R???", their_call, YOUR_CALL);
    ft8_qso.rows[3].is_tx = true;
    ft8_qso.rows[3].state = FT8_QSO_ROW_PENDING;

    // Row 4 RX: their RR73
    snprintf(ft8_qso.rows[4].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s RR73", YOUR_CALL, their_call);
    ft8_qso.rows[4].is_tx = false;
    ft8_qso.rows[4].state = FT8_QSO_ROW_PENDING;

    // Row 5 TX: our 73
    snprintf(ft8_qso.rows[5].text, FTX_MAX_DISPLAY_LENGTH,
      "%s %s 73", their_call, YOUR_CALL);
    ft8_qso.rows[5].is_tx = true;
    ft8_qso.rows[5].state = FT8_QSO_ROW_PENDING;

    ft8_qso.num_rows = 6;
  }

  // test response encoding is valid
  uint8_t test_tones[FT8_NN];
  memset(test_tones, 0, sizeof(test_tones));
  if (!ft8_encode_message(ft8_qso.rows[1].text, test_tones))
  {
    memset(&ft8_qso, 0, sizeof(ft8_qso));
    ft8_set_popup("Cannot encode","ERROR:");
    LOG(LOG_WARN, "Cannot encode response: %s\n", ft8_qso.rows[1].text);
    return;
  }

  ft8_qso.current_step = 1;
  ft8_qso.active = true;
  ft8_qso.is_direct = is_direct;

  LOG(LOG_INFO, "QSO start (%s): %s at %.0f Hz extra=[%s]\n",
    is_cq ? "CQ-resp" : "direct",
    their_call, freq, their_extra);
}

//------------------------------------------------------------------------------
// ft8_qso_get_tx_message()
// Returns message text for the current TX step, or NULL if not a TX step.
//------------------------------------------------------------------------------
static const char* ft8_qso_get_tx_message(void)
{
  if (!ft8_qso.active) return NULL;
  const int s = ft8_qso.current_step;
  if (s < 0 || s >= FT8_QSO_NUM_ROWS) return NULL;
  if (!ft8_qso.rows[s].is_tx) return NULL;
  return ft8_qso.rows[s].text;
}

//------------------------------------------------------------------------------
// ft8_qso_after_tx()
// Marks the current TX step DONE and advances to the next step.
// Sets ft8_qso.active = false when all steps complete.
//------------------------------------------------------------------------------
static void ft8_qso_after_tx(void)
{
  if (!ft8_qso.active) return;
  ft8_qso.rows[ft8_qso.current_step].state = FT8_QSO_ROW_DONE;
  ft8_qso.current_step++;
  if (ft8_qso.current_step >= ft8_qso.num_rows)
  {
    ft8_qso.active = false;
    ft8_qso_display_until = millis() + 2000ul;
    ft8_set_popup(ft8_qso.their_call,"COOL:");
    LOG(LOG_INFO, "QSO complete\n");
    return;
  }
  ft8_qso.rows[ft8_qso.current_step].state = FT8_QSO_ROW_CURRENT;
}

//------------------------------------------------------------------------------
// ft8_qso_check_rx()
// Called after each decode when QSO is active and current step is an RX step.
// Scans decoded lines for their callsign + our callsign.
// On match: updates the row text, advances to next step.
// On miss:  increments attempts, updates row state (yellow→orange→red).
//------------------------------------------------------------------------------
static void ft8_qso_check_rx(
  const char lines[][FTX_MAX_DISPLAY_LENGTH],
  const uint32_t count)
{
  if (!ft8_qso.active) return;
  const int step = ft8_qso.current_step;
  if (step < 0 || step >= ft8_qso.num_rows) return;
  ft8_qso_row_t* row = &ft8_qso.rows[step];
  if (row->is_tx) return;
  for (uint32_t i = 0; i < count; i++)
  {
    float snr = 0.0f, dt = 0.0f, freq = 0.0f;
    char msg[FTX_MAX_MESSAGE_LENGTH] = "";
    if (sscanf(lines[i], "%f %f %f %34[^\n]", &snr, &dt, &freq, msg) < 4) continue;

    if (!strstr(msg, ft8_qso.their_call)) continue;
    if (!strstr(msg, YOUR_CALL)) continue;

    const char* last_sp = strrchr(msg, ' ');
    const char* token = last_sp ? last_sp + 1 : msg;

    // ── Step-specific validation ──────────────────────────────────────
    if (ft8_qso.is_direct && step == 2)
    {
      // R3: accept R+report (any) or RR73/RRR/73
      const bool is_r_report = (token[0] == 'R' &&
        (token[1] == '-' || token[1] == '+'));
      const bool is_end = (strcmp(token, "RR73") == 0 ||
        strcmp(token, "RRR")  == 0 ||
        strcmp(token, "73")   == 0);
      if (!is_r_report && !is_end) continue;

      // Update T4 message based on what we received
      if (is_r_report)
        snprintf(ft8_qso.rows[3].text, FTX_MAX_DISPLAY_LENGTH,
          "%s %s RRR", ft8_qso.their_call, YOUR_CALL);
      else
        snprintf(ft8_qso.rows[3].text, FTX_MAX_DISPLAY_LENGTH,
          "%s %s 73", ft8_qso.their_call, YOUR_CALL);
    }
    else if (!ft8_qso.is_direct && step == 4)
    {
      // CQ-resp R5: must be RR73/RRR/73
      if (strcmp(token, "RR73") != 0 &&
        strcmp(token, "RRR") != 0 &&
        strcmp(token, "73") != 0) continue;
    }
    // other steps: any message with both callsigns accepted

    // ── Good match ────────────────────────────────────────────────────
    strncpy(row->text, msg, FTX_MAX_DISPLAY_LENGTH - 1);
    row->state = FT8_QSO_ROW_DONE;

    // CQ-response R3 (step 2): extract their report → build R+report for T4
    if (!ft8_qso.is_direct && step == 2 && last_sp && last_sp[1] != '\0')
    {
      snprintf(ft8_qso.r_report, sizeof(ft8_qso.r_report),
        "R%s", last_sp + 1);
      snprintf(ft8_qso.rows[3].text, FTX_MAX_DISPLAY_LENGTH,
        "%s %s %s",
        ft8_qso.their_call, YOUR_CALL, ft8_qso.r_report);
    }

    // Advance
    ft8_qso.current_step++;
    if (ft8_qso.current_step < ft8_qso.num_rows)
      ft8_qso.rows[ft8_qso.current_step].state = FT8_QSO_ROW_CURRENT;
    else
    {
      ft8_qso.active = false;
      ft8_selected.valid = false;
      ft8_ui_state = FT8_UI_AUTO;
      ft8_qso_display_until = millis() + 2000ul;
      ft8_set_popup(ft8_qso.their_call,"COOL:");
    }

    LOG(LOG_INFO, "QSO step %d done: %s\n", step, msg);
    return;
  }

  // ── No reply this slot ────────────────────────────────────────────────
  row->attempts++;

  // Final RX step: exit gracefully on timeout rather than retransmitting forever
  const bool is_final_rx = (ft8_qso.current_step == ft8_qso.num_rows - 1);
  if (is_final_rx && row->attempts >= FT8_QSO_MAX_RETRIES)
  {
    row->state = FT8_QSO_ROW_DONE;   // mark done anyway
    ft8_qso.active = false;
    ft8_selected.valid = false;
    ft8_ui_state = FT8_UI_AUTO;    
    ft8_qso_display_until = millis() + 2000ul;
    ft8_set_popup(ft8_qso.their_call,"COOL:");
    LOG(LOG_INFO, "QSO complete (R5 timeout - exiting)\n");
    return;
  }

  if (row->attempts >= FT8_QSO_MAX_RETRIES) row->state = FT8_QSO_ROW_TIMEOUT;
  else if (row->attempts >= 2) row->state = FT8_QSO_ROW_RETRY;
  else row->state = FT8_QSO_ROW_WAITING;

  LOG(LOG_INFO, "QSO step %d: no reply attempt %d\n", step, row->attempts);
}
//------------------------------------------------------------------------------
// ft8_qso_abort()
// Clean abort — returns to history browse, stays in FT8 mode.
//------------------------------------------------------------------------------
static void ft8_qso_abort(void)
{
  LOG(LOG_INFO, "QSO aborted at step %d\n", ft8_qso.current_step);
  memset(&ft8_qso, 0, sizeof(ft8_qso));
  memset(&ft8_cq, 0, sizeof(ft8_cq));
  ft8_selected.valid = false;
  ft8_ui_state = FT8_UI_AUTO;
}

static void ft8_init(void)
{
  memset(ft8_history, 0, sizeof(ft8_history));
  memset(ft8_display_buf, 0, sizeof(ft8_display_buf));
  memset(&ft8_qso, 0, sizeof(ft8_qso));
  memset(&ft8_cq, 0, sizeof(ft8_cq));
  ft8_history_count = 0;
  ft8_history_head = 0;
  ft8_display_count = 0;
  ft8_selected.valid = false;
  ft8_ui_state = FT8_UI_AUTO;
  ft8_new_available = false;
  ft8_exit_pending_until = 0;
}

//------------------------------------------------------------------------------
// MAIN FT8 FUNCTION
// Called from loop1() on core 1 while radio.mode == MODE_FT8
// Returns false to exit FT8 mode
//------------------------------------------------------------------------------
static const bool do_ft8(void)
{
  // Persistent state across calls
  static uint32_t slot_calibrate_ms = 0;
  static uint32_t ft8_button_down_time = 0;
  static uint32_t progress = 0;
  static ft8_state_t ft8_state = FT8_STATE_WAITING;
  static bool ft8_hash_init = false;

  //--------------------------------------------------------------------------
  // One-time calibration on first entry
  // User presses button at a known 15-second slot boundary
  //--------------------------------------------------------------------------
  if (slot_calibrate_ms == 0)
  {
    delay(50);
    // wait for release
    while (digitalRead(PIN_ENCBUT) == LOW)
      delay(50);

    uint32_t cal_progress = 0;
    uint32_t disp_update  = 0;
    const uint32_t cal_start = millis();
    while (digitalRead(PIN_ENCBUT) == HIGH)
    {
      const uint32_t now = millis();
      // give them minute!
      if (now - cal_start > 60000ul) return false;
      if (now > disp_update)
      {
        disp_update = now + 50ul;
        ft8_display(0, ft8_state, cal_progress++, true);
        if (cal_progress > 240) cal_progress = 0;
      }
    }
    slot_calibrate_ms = millis();
    delay(50);
    // wait for release
    while (digitalRead(PIN_ENCBUT) == LOW)
      delay(50);
  }

  //--------------------------------------------------------------------------
  // One-time hash table init
  //--------------------------------------------------------------------------
  if (!ft8_hash_init)
  {
    ft8_init();
    ft8_hashtable_init();
    ft8_hash_init = true;
  }

  //--------------------------------------------------------------------------
  // Slot timing
  //--------------------------------------------------------------------------
  const uint32_t now = millis();
  const uint32_t slot_ms = (now - slot_calibrate_ms) % FT8_SLOT_MS;
  const uint32_t slot_num = (now - slot_calibrate_ms) / FT8_SLOT_MS;
  const bool even_slot = (slot_num % 2) == 0;

  //--------------------------------------------------------------------------
  // Handle rotary encoder input
  //--------------------------------------------------------------------------
  ft8_handle_rotary();

  //--------------------------------------------------------------------------
  // Auto-return to live view after inactivity in browse mode
  //--------------------------------------------------------------------------
  // FT8_UI_SELECTED persists until TX fires or user presses button to cancel
  if (ft8_ui_state == FT8_UI_BROWSE)
  {
    if ((now - ft8_last_interaction) > FT8_BROWSE_TIMEOUT_MS)
    {
      ft8_ui_state = FT8_UI_AUTO;
      ft8_cursor = 0;
    }
  }
  //--------------------------------------------------------------------------
  // FT8 receive / decode state machine
  //--------------------------------------------------------------------------
  switch (ft8_state)
  {
    case FT8_STATE_WAITING:
    {
      if (slot_ms < FT8_START_THRESHOLD)
      {
        bool should_tx = false;
        if (ft8_qso.active)
        {
          const int step = ft8_qso.current_step;
          if (step < 0 || step >= ft8_qso.num_rows)
          {
            // invariant broken - abort cleanly
            ft8_set_popup("QSO error", "ERROR:");
            ft8_qso_abort();
            ft8_state = FT8_STATE_WAITING;
            break;
          }          
          const ft8_qso_row_t* row = &ft8_qso.rows[step];
          if (row->is_tx)
          {
            // TX step — fire in correct parity slot
            should_tx = (ft8_selected.respond_in_even_slot == even_slot);
          }
          else
          {
            // RX step — retransmit previous TX in our TX slots
            // until they reply (DONE) or user aborts
            // TIMEOUT is visual only — keep retransmitting until abort
            should_tx = (row->state != FT8_QSO_ROW_DONE) &&
              (ft8_selected.respond_in_even_slot == even_slot);
          }
        }
        else if (ft8_cq.active)
        {
          should_tx = (ft8_cq.even_slot == even_slot);
        }
        else
        {
          should_tx = ft8_selected.valid &&
            (ft8_selected.respond_in_even_slot == even_slot);
        }
        if (should_tx)
        {
          ft8_state = FT8_STATE_TRANSMITTING;
          ft8_button_down_time = 0;
          progress  = 0;
        }
        else
        {
          progress = 0;
          ft8_monitor_init(&ft8mon);
          ft8_state = FT8_STATE_RECEIVING;
        }
      }
      break;
    }
    case FT8_STATE_RECEIVING:
    {
      static const uint32_t buff_size = FT8_FFT_SIZE / 2;  // 1250
      static uint32_t last_offset = UINT32_MAX;

      // Reset last_offset when re-entering receiving state
      if (slot_ms < FT8_START_THRESHOLD + 50u) last_offset = UINT32_MAX;

      const uint32_t sample_p = ft8_sample_p;
      uint32_t offset = 0;
      bool valid = true;

      if (sample_p < 400u)
        // second half complete
        offset = buff_size;
      else if (sample_p >= buff_size && sample_p < (buff_size + 400u))
        // first half complete
        offset = 0;
      else
        // dead zone
        valid = false;

      if (valid && offset != last_offset)
      {
        last_offset = offset;
        ft8_monitor_process(&ft8mon, (const float*)ft8_data + offset);
        progress++;
      }

      if (slot_ms >= FT8_RECEIVE_MS)
        ft8_state = FT8_STATE_DECODING;

      break;
    }
    case FT8_STATE_DECODING:
    {
      const uint32_t decode_start = millis();
      progress = 0;
      ft8_cal_freeze = slot_calibrate_ms;
      static char ft8_lines[FT8_MAX_DECODED][FTX_MAX_DISPLAY_LENGTH] = { 0 };
      memset(ft8_lines, 0, sizeof(ft8_lines));
      const uint32_t n = ft8_decode(&ft8mon, ft8_lines, ft8_ui_callback);
      ft8_history_add(ft8_lines, n, even_slot, (uint8_t)slot_num);

      if (ft8_qso.active) ft8_qso_check_rx(ft8_lines, n);

      if (ft8_ui_state == FT8_UI_AUTO) ft8_new_available = false;
      ft8_monitor_reset(&ft8mon);

      // compensate button timer for the blocking decode period
      if (ft8_button_down_time != 0 && ft8_button_down_time < decode_start)
      {
        const uint32_t decode_duration = millis() - decode_start;
        ft8_button_down_time += decode_duration;
      }

      ft8_state = FT8_STATE_WAITING;
      break;
    }
    case FT8_STATE_TRANSMITTING:
    {
      if (ft8_qso.active)
      {
        // Determine which message to send:
        // If current step is RX (retransmit case), send previous TX step message
        const int tx_step = ft8_qso.rows[ft8_qso.current_step].is_tx?
          ft8_qso.current_step : ft8_qso.current_step - 1;
        const char* msg = ft8_qso.rows[tx_step].text;
        uint8_t tones[FT8_NN];
        memset(tones, 0, sizeof(tones));
        if (ft8_encode_message(msg, tones))
        {
          const float tone0 = (float)tones[0] * FT8_TONE_SPACING;
          set_frequency((uint32_t)((ft8_qso.audio_freq + tone0) * 100.0f));
          ft8_enable_tx();
          const bool ok = ft8_transmit_tones(tones, ft8_qso.audio_freq, slot_calibrate_ms, ft8_state);
          ft8_disable_tx();
          if (!ok)
          {
            ft8_qso_abort();
            ft8_state = FT8_STATE_WAITING;
            break;
          }
          if (ft8_qso.rows[ft8_qso.current_step].is_tx)
          {
            // First transmission of this TX step — advance to RX step
            ft8_qso_after_tx();
          }
          else
          {
            // Retransmit — count as an attempt on the RX step
            // (attempt already incremented by ft8_qso_check_rx on missed slot)
            LOG(LOG_INFO, "QSO retransmit step %d, attempt %d\n",
              tx_step, ft8_qso.rows[ft8_qso.current_step].attempts);
          }
        }
        else
        {
          LOG(LOG_ERROR, "QSO TX encode failed: %s\n", msg);
          ft8_set_popup("Encode failed","ERROR:");
          ft8_qso_abort();
        }
        if (!ft8_qso.active)
        {
          ft8_selected.valid = false;
          ft8_ui_state = FT8_UI_AUTO;
        }
      }
      else if (ft8_cq.active)
      {
        if (!ft8_cq_transmit(slot_calibrate_ms, ft8_state))
          // long-press abort or encode err
          ft8_cq.active = false;
      }
      else
      {
        // should not happen
        ft8_selected.valid = false;
        ft8_ui_state = FT8_UI_AUTO;
      }
      ft8_state = FT8_STATE_WAITING;
      break;
    }
  }

  //--------------------------------------------------------------------------
  // Handle button press (exit mode or browse/select)
  //--------------------------------------------------------------------------
  // Non-blocking button press handling
  // Records press start time; processes the action on release.
  // The receive state machine continues to run between samples.
  //--------------------------------------------------------------------------
  if (ft8_state != FT8_STATE_TRANSMITTING)
  {
    const bool button_pressed = (digitalRead(PIN_ENCBUT) == LOW);
    if (ft8_button_down_time == 0 && button_pressed)
    {
      // press starting — record time and continue
      ft8_button_down_time = millis();
    }
    else if (ft8_button_down_time != 0 && !button_pressed)
    {
      // released — measure and act
      const uint32_t held_ms = millis() - ft8_button_down_time;
      // reset BEFORE any return false
      ft8_button_down_time = 0;
      if (held_ms < 50)
      {
        // debounce — ignore
      }
      else if (held_ms >= FT8_BUTTON_LONG_PRESS)
      {
        // long press — abort current activity (priority: QSO → CQ → exit)
        progress = 0;
        if (ft8_qso.active)
        {
          ft8_exit_pending_until = 0;
          ft8_qso_abort();
          ft8_state = FT8_STATE_WAITING;
        }
        else if (ft8_cq.active)
        {
          ft8_exit_pending_until = 0;
          ft8_cq.active = false;
          ft8_state = FT8_STATE_WAITING;
          LOG(LOG_INFO, "CQ stopped\n");
        }
        else if (ft8_selected.valid)
        {
          // cancel pending single TX
          ft8_exit_pending_until = 0;
          ft8_selected.valid = false;
          ft8_ui_state = FT8_UI_AUTO;
          ft8_state = FT8_STATE_WAITING;
        }
        else
        {
          // Exit FT8 requires confirmation - long press twice within 5s
          const uint32_t now_ms = millis();
          if (ft8_exit_pending_until != 0 && now_ms < ft8_exit_pending_until)
          {
              // Confirmed
              ft8_init();
              ft8_state = FT8_STATE_WAITING;
              return false;
          }
          // First long press - request confirmation
          ft8_exit_pending_until = now_ms + 5000ul;
          ft8_set_popup("Hold to exit","CONFIRM EXIT:");
        }
      }
      else
      {
        // short press — action depends on UI state
        ft8_last_interaction = millis();
        switch (ft8_ui_state)
        {
          case FT8_UI_AUTO:
          {
            ft8_cq_start(slot_calibrate_ms);
            break;
          }
          case FT8_UI_BROWSE:
          {
            // don't replace an active QSO
            if (ft8_qso.active) break;
            if (ft8_display_count > 0 &&
              ft8_cursor >= 0 &&
              ft8_cursor < (int32_t)ft8_display_count)
            {
              ft8_cq.active = false;
              const ft8_history_entry_t* e = &ft8_display_buf[ft8_cursor];
              memcpy(ft8_selected.text, e->text, FTX_MAX_DISPLAY_LENGTH);
              ft8_selected.respond_in_even_slot = !e->received_in_even_slot;
              ft8_selected.slot_number_low = e->slot_number_low;
              ft8_selected.valid = false;
              ft8_qso_start();
              if (ft8_qso.active)
              {
                // only valid if QSO actually started
                ft8_selected.valid = true;       
                ft8_ui_state = FT8_UI_SELECTED;
              }
              else
              {
                ft8_set_popup("Not CQ/Your Call","INFO");
              }
            }
            break;
          }
          case FT8_UI_SELECTED:
          {
            break;
          }
        }
      }
    }
    // Otherwise: button still up (do nothing) or still down (keep tracking)
  }
  //--------------------------------------------------------------------------
  // Update display every 50ms
  //--------------------------------------------------------------------------
  static uint32_t next_update = 0;
  if (millis() > next_update)
  {
    next_update = millis() + 50ul;
    const uint32_t prog = UTIL::map(progress, 0, 80, 0, 240);
    ft8_display(slot_calibrate_ms, ft8_state, prog);
  }

  return true;  // stay in FT8 mode
}

/*
 * general UI processing
 */
void loop1(void)
{
  // run UI on core 1
  if (radio.mode == MODE_FT8)
  {
    // keep FT8 completely separate
    if (do_ft8())
    {
      return;
    }
    radio.mode_auto = true;
    radio.mode = get_mode_auto();
    set_frequency();
  }
  static uint32_t old_frequency = radio.frequency;
  static uint32_t old_band = radio.band;
  static uint32_t old_sidetone = radio.sidetone;
  static uint32_t old_cw_level = radio.cw_level;
  static uint32_t old_cw_dit = radio.cw_dit;
  static uint32_t old_spectype = radio.spectype;
  static uint32_t old_jnrlevel = radio.jnrlevel;
  static uint32_t old_micgain = radio.micgain;
  static uint32_t old_bandwidth = radio.bandwidth;
  static int8_t old_level = 0;
  static bool old_cessb = radio.cessb;
  static bool old_graph_swr = radio.graph_swr;
  static mode_t old_mode = radio.mode;

  // process button press
  // short press: change step
  // long press: menu options
  enum button_state_t {BUTTON_IDLE,BUTTON_TEST_SHORT,BUTTON_WAIT_RELEASE};
  enum button_action_t {BUTTON_NO_PRESS,BUTTON_SHORT_PRESS,BUTTON_LONG_PRESS};
  static button_state_t button_state = BUTTON_IDLE;
  static uint32_t button_timer = 0;
  button_action_t button_action = BUTTON_NO_PRESS;
  if (!radio.tx_enable)
  {
    // receive mode
    if (radio.menu_active)
    {
      const option_value_t option = process_menu();
      switch (option)
      {
        case OPTION_MODE_LSB:        radio.mode = MODE_LSB; radio.mode_auto = false; break;
        case OPTION_MODE_USB:        radio.mode = MODE_USB; radio.mode_auto = false; break;
        case OPTION_MODE_CWL:        radio.mode = MODE_CWL; radio.mode_auto = false; break;
        case OPTION_MODE_CWU:        radio.mode = MODE_CWU; radio.mode_auto = false; break;
        case OPTION_MODE_DGL:        radio.mode = MODE_DGL; radio.mode_auto = false; break;
        case OPTION_MODE_DGU:        radio.mode = MODE_DGU; radio.mode_auto = false; break;
        case OPTION_MODE_FT8:        radio.mode = MODE_FT8; radio.mode_auto = false; break;
        case OPTION_MODE_AM:         radio.mode = MODE_AM;  radio.mode_auto = false; break;
        case OPTION_MODE_AUTO:       radio.mode_auto = true;                         break;
        case OPTION_STEP_10:         radio.step = 10U;                               break;
        case OPTION_STEP_100:        radio.step = 100U;                              break;
        case OPTION_STEP_500:        radio.step = 500U;                              break;
        case OPTION_STEP_1000:       radio.step = 1000U;                             break;
        case OPTION_STEP_5000:       radio.step = 5000U;                             break;
        case OPTION_STEP_10000:      radio.step = 10000U;                            break;
        case OPTION_STEP_100000:     radio.step = 100000U;                           break;
        case OPTION_BAND_80M:        radio.band = BAND_80M;                          break;
        case OPTION_BAND_40M:        radio.band = BAND_40M;                          break;
        case OPTION_BAND_30M:        radio.band = BAND_30M;                          break;
        case OPTION_BAND_20M:        radio.band = BAND_20M;                          break;
        case OPTION_BAND_17M:        radio.band = BAND_17M;                          break;
        case OPTION_BAND_15M:        radio.band = BAND_15M;                          break;
        case OPTION_BAND_12M:        radio.band = BAND_12M;                          break;
        case OPTION_BAND_10M:        radio.band = BAND_10M;                          break;
        case OPTION_BAND_SWL:        radio.band = BAND_SWL;                          break;
        case OPTION_BW_2000:         radio.bandwidth = 1ul;                          break;
        case OPTION_BW_2200:         radio.bandwidth = 2ul;                          break;
        case OPTION_BW_2400:         radio.bandwidth = 3ul;                          break;
        case OPTION_BW_2600:         radio.bandwidth = 4ul;                          break;
        case OPTION_BW_2800:         radio.bandwidth = 5ul;                          break;
        case OPTION_SIDETONE_500:    radio.sidetone = 500u;                          break;
        case OPTION_SIDETONE_550:    radio.sidetone = 550u;                          break;
        case OPTION_SIDETONE_600:    radio.sidetone = 600u;                          break;
        case OPTION_SIDETONE_650:    radio.sidetone = 650u;                          break;
        case OPTION_SIDETONE_700:    radio.sidetone = 700u;                          break;
        case OPTION_SIDETONE_750:    radio.sidetone = 750u;                          break;
        case OPTION_SIDETONE_800:    radio.sidetone = 800u;                          break;
        case OPTION_SIDETONE_850:    radio.sidetone = 850u;                          break;
        case OPTION_SIDETONE_LOW:    radio.cw_level = 1u;                            break;
        case OPTION_SIDETONE_MED:    radio.cw_level = 2u;                            break;
        case OPTION_SIDETONE_HI:     radio.cw_level = 3u;                            break;
        case OPTION_CW_SPEED_10:     radio.cw_dit = 120u;                            break;
        case OPTION_CW_SPEED_15:     radio.cw_dit = 80u;                             break;
        case OPTION_CW_SPEED_20:     radio.cw_dit = 60u;                             break;
        case OPTION_CW_SPEED_25:     radio.cw_dit = 48u;                             break;
        case OPTION_CW_SPEED_30:     radio.cw_dit = 40u;                             break;
        case OPTION_DECODE_ADAPTIVE: radio.cwdecode = 1;                             break;
        case OPTION_DECODE_SCHMITT:  radio.cwdecode = 2;                             break;
        case OPTION_CWDECODE_OFF:    radio.cwdecode = 0;                             break;
        case OPTION_SPECTRUM_WIND:   radio.spectype = SPECTRUM_WIND;                 break;
        case OPTION_SPECTRUM_GRASS:  radio.spectype = SPECTRUM_GRASS;                break;
        case OPTION_SPEC_SETLEVEL:   set_spectrum_level = true;                      break;
        case OPTION_SPEC_ADJLEVEL:   adj_spectrum_level = true;                      break;
        case OPTION_JNR_LEVEL1:      radio.jnrlevel = JNR_LEVEL1;                    break;
        case OPTION_JNR_LEVEL2:      radio.jnrlevel = JNR_LEVEL2;                    break;
        case OPTION_JNR_LEVEL3:      radio.jnrlevel = JNR_LEVEL3;                    break;
        case OPTION_JNR_OFF:         radio.jnrlevel = JNR_OFF;                       break;
        case OPTION_NB_LEVEL1:       radio.nblevel = NB_LEVEL1;                      break;
        case OPTION_NB_LEVEL2:       radio.nblevel = NB_LEVEL2;                      break;
        case OPTION_NB_LEVEL3:       radio.nblevel = NB_LEVEL3;                      break;
        case OPTION_NB_LEVEL4:       radio.nblevel = NB_LEVEL4;                      break;
        case OPTION_NB_LEVEL5:       radio.nblevel = NB_LEVEL5;                      break;
        case OPTION_NB_OFF:          radio.nblevel = NB_OFF;                         break;
        case OPTION_MIC_25:          radio.micgain = 25;                             break;
        case OPTION_MIC_50:          radio.micgain = 50;                             break;
        case OPTION_MIC_75:          radio.micgain = 75;                             break;
        case OPTION_MIC_100:         radio.micgain = 100;                            break;
        case OPTION_MIC_125:         radio.micgain = 125;                            break;
        case OPTION_MIC_150:         radio.micgain = 150;                            break;
        case OPTION_MIC_175:         radio.micgain = 175;                            break;
        case OPTION_MIC_200:         radio.micgain = 200;                            break;
        case OPTION_MIC_PROC1:       radio.micproc = MIC_PROC1;                      break;
        case OPTION_MIC_PROC2:       radio.micproc = MIC_PROC2;                      break;
        case OPTION_MIC_PROC3:       radio.micproc = MIC_PROC3;                      break;
        case OPTION_MIC_PROC4:       radio.micproc = MIC_PROC4;                      break;
        case OPTION_MIC_PROC5:       radio.micproc = MIC_PROC5;                      break;
        case OPTION_MIC_PROC_OFF:    radio.micproc = MIC_PROC_OFF;                   break;
        case OPTION_CESSB_ON:        radio.cessb = true;                             break;
        case OPTION_CESSB_OFF:       radio.cessb = false;                            break;
        case OPTION_GAUSSIAN_ON:     radio.gaussian = true;                          break;
        case OPTION_GAUSSIAN_OFF:    radio.gaussian = false;                         break;
        case OPTION_GRAPH_SWR_Y:     radio.graph_swr = true;                         break;
        case OPTION_GRAPH_SWR_N:     radio.graph_swr = false;                        break;
        case OPTION_EXIT:            radio.menu_active = false;                      break;
      }

      if (set_spectrum_level)
      {
        // restore the saved level
        radio.level[radio.band] = radio.slevel[radio.band];
        old_level = radio.level[radio.band];
      }
      if (adj_spectrum_level)
      {
        // only changing current level, not saving it
        old_level = radio.level[radio.band];
      }

      // when settings change save them
      bool settings_changed = false;

      // update DDS phase if sidetone changed
      if (radio.sidetone != old_sidetone)
      {
        old_sidetone = radio.sidetone;
        settings_changed = true;
        radio.cw_phase = ((uint64_t)radio.sidetone * (1ull << 32)) / SAMPLERATE;
      }

      // CW timing
      if (radio.cw_dit != old_cw_dit)
      {
        old_cw_dit = radio.cw_dit;
        settings_changed = true;
      }

      // CW level
      if (radio.cw_level != old_cw_level)
      {
        old_cw_level = radio.cw_level;
        settings_changed = true;
      }

      // spectrum type
      if (radio.spectype != old_spectype)
      {
        old_spectype = radio.spectype;
        settings_changed = true;
      }

      // JNR level
      if (radio.jnrlevel != old_jnrlevel)
      {
        old_jnrlevel = radio.jnrlevel;
        settings_changed = true;
      }

      // mic gain
      if (radio.micgain != old_micgain)
      {
        old_micgain = radio.micgain;
        settings_changed = true;
      }

      // bandwidth
      if (radio.bandwidth != old_bandwidth)
      {
        old_bandwidth = radio.bandwidth;
        settings_changed = true;
      }

      // CESSB
      if (radio.cessb != old_cessb)
      {
        old_cessb = radio.cessb;
        settings_changed = true;
      }

      // graph SWR
      if (radio.graph_swr != old_graph_swr)
      {
        old_graph_swr = radio.graph_swr;
        settings_changed = true;
      }

      // save the settings
      if (settings_changed)
      {
        save_settings_now = true;
        while (save_settings_now)
        {
          // EEPROM will pause this core
          // do nothing until saved
          tight_loop_contents();
        }
      }

      // mode may change in auto
      const radio_mode_t auto_mode = get_mode_auto();
      if (radio.mode != auto_mode)
      {
        radio.mode = auto_mode;
      }

      // update frequency if mode changes
      // delay enabling Mic for Vox
      if (radio.mode!=old_mode)
      {
        old_mode = radio.mode;
        vox_mic_ready = false;
        vox_triggered = false;
        if (radio.mode==MODE_DGL || radio.mode==MODE_DGU)
        {
          enable_mic();
          delay(100);
          vox_mic_ready = true;
          radio.tx_safe = true;
        }
        else
        {
          disable_mic();
        }
        set_frequency();
      }

      // set band
      if (radio.band!=old_band)
      {
        save_data[old_band].frequency = radio.frequency;
        radio.frequency = save_data[radio.band].frequency;
        old_frequency = radio.frequency;
        old_band = radio.band;
        // mute during change
        mute();
        delay(50);
        set_filter();
        set_frequency();
        delay(50);
      }
    }
    else
    {
      // menu not active
      if (set_spectrum_level || adj_spectrum_level)
      {
        // rotary sets level
        mutex_enter_blocking(&rotary_mutex);
        const int32_t level_delta = radio.tune;
        radio.tune = 0;
        mutex_exit(&rotary_mutex);
        int8_t new_level = radio.level[radio.band] + level_delta;
        new_level = constrain(new_level,SPECTRUM_LEVEL_MIN,SPECTRUM_LEVEL_MAX);
        radio.level[radio.band] = new_level;
        if (digitalRead(PIN_ENCBUT)==LOW)
        {
          // if button pressed
          // save new level (if changed)
          // wait for button release
          if (set_spectrum_level && new_level!=old_level)
          {
            // remember saved level
            radio.slevel[radio.band] = new_level;
            save_settings_now = true;
            while (save_settings_now)
            {
              // EEPROM will pause this core
              // do nothing until saved
              tight_loop_contents();
            }
          }
          delay(50);
          while (digitalRead(PIN_ENCBUT)==LOW)
          {
            tight_loop_contents();
          }
          delay(50);
          set_spectrum_level = false;
          adj_spectrum_level = false;
        }
      }
      else
      {
        switch (button_state)
        {
          case BUTTON_IDLE:
          {
            if (digitalRead(PIN_ENCBUT)==LOW)
            {
              button_state = BUTTON_TEST_SHORT;
              button_timer = millis()+BUTTON_LONG_PRESS_TIME;
              delay(50);
            }
            break;
          }
          case BUTTON_TEST_SHORT:
          {
            const uint32_t now = millis();
            if (digitalRead(PIN_ENCBUT)==HIGH)
            {
              button_state = BUTTON_IDLE;
              if (now<button_timer)
              {
                button_action = BUTTON_SHORT_PRESS;
              }
              delay(50);
              break;
            }
            if (now>button_timer)
            {
              button_state = BUTTON_WAIT_RELEASE;
              button_action = BUTTON_LONG_PRESS;
            }
            break;
          }
          case BUTTON_WAIT_RELEASE:
          {
            if (digitalRead(PIN_ENCBUT)==HIGH)
            {
              button_state = BUTTON_IDLE;
              delay(50);
            }
            break;
          }
        }

        // process button action
        switch (button_action)
        {
          case BUTTON_SHORT_PRESS:
          {
            radio.step *= 10u;
            if (radio.step>1000u) radio.step = 100u;
            break;
          }
          case BUTTON_LONG_PRESS:
          {
            init_menu();
            radio.menu_active = true;
            break;
          }
        }

        // set the mode based on frequency if auto
        const radio_mode_t auto_mode = get_mode_auto();
        if (radio.mode != auto_mode)
        {
          radio.mode = auto_mode;
        }

        // process main tuning
        mutex_enter_blocking(&rotary_mutex);
        const int32_t tuning_delta = radio.tune;
        radio.tune = 0;
        mutex_exit(&rotary_mutex);
        uint32_t new_frequency = radio.frequency;
        new_frequency = new_frequency+(tuning_delta * (int32_t)radio.step);
        new_frequency = new_frequency/radio.step;
        new_frequency = new_frequency*radio.step;
        new_frequency = constrain(new_frequency,bands[radio.band].lo,bands[radio.band].hi);
        if (new_frequency!=old_frequency || radio.mode!=old_mode)
        {
          radio.frequency = new_frequency;
          old_frequency = new_frequency;
          old_mode = radio.mode;
          set_frequency();
          if (radio.band==BAND_SWL)
          {
            set_filter();
          }
        } // frequency change
      } // spectrum level
    } // menu active
  } // receive mode
 
  // unmute in case muted
  unmute();

  // process spectrum
  static uint32_t process_spectrum_txrx = 0;
  if (millis()>process_spectrum_txrx)
  {
    process_spectrum();
  }

  // update the display every 50ms
  static uint32_t next_update = 0;
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update = now + 50ul;
    update_display(DSP::smeter(radio.frequency));
  }

  // check for PTT
  const bool digital = (radio.mode==MODE_DGL || radio.mode==MODE_DGU || radio.mode==MODE_FT8);
  const bool b_PTT = (!digital && radio.tx_safe && digitalRead(PIN_PTT)==LOW);
  const bool b_PADA = (!digital && radio.tx_safe && digitalRead(PIN_PADA)==LOW);
  const bool b_PADB = (!digital && radio.tx_safe && digitalRead(PIN_PADB)==LOW);
  radio.tx_button = (b_PTT || b_PADA || b_PADB);
  if ((radio.band==BAND_SWL) || (radio.mode==MODE_AM))
  {
    // capture TX button before returning
    return;
  }
  if (radio.tx_button || vox_triggered)
  {
    const float saved_agc = DSP::agc_peak;
    radio.menu_active = false;
    if (set_spectrum_level || adj_spectrum_level)
    {
      // restore old level in case it was changed
      set_spectrum_level = false;
      adj_spectrum_level = false;
      radio.level[radio.band] = old_level;
    }
    if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
    {
      process_key();
    }
    else if (b_PTT)
    {
      enable_ssb_tx();
      ptt_release();
    }
    else if (vox_triggered)
    {
      // wait for Vox to time out
      vox_triggered = false;
      enable_ssb_tx();
      uint32_t now = millis();
      uint32_t vox_timeout = now + VOX_TIMEOUT;
      while (now < vox_timeout)
      {
        process_vox_tx();
        now = millis();
        if (vox_triggered)
        {
          vox_triggered = false;
          vox_timeout = now + VOX_TIMEOUT;
        }
      }
    }
    if (vox_triggered)
    {
      vox_triggered = false;
    }

    // back to receive
    memset(magnitude,0,sizeof(magnitude));
    process_spectrum_txrx = millis()+200;
    lpf_port.output(I2C_PIN_TXENABLE,TCA9534::Level::L);
    delay(50);
    radio.tx_enable = false;
    digitalWrite(PIN_TXBIAS,LOW);
    bpf_port.output(I2C_PIN_TXN,TCA9534::Level::H);
    bpf_port.output(I2C_PIN_RXN,TCA9534::Level::L);
    set_frequency();
    update_display();
    unmute();
    digitalWrite(LED_BUILTIN,LOW);
    DSP::agc_peak = saved_agc;
  }
}