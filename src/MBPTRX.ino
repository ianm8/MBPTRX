/*
 * MBPTRX Version 2.4.240
 *
 * Copyright 2025 Ian Mitchell VK7IAN
 * Licenced under the GNU GPL Version 3
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
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

//#define YOUR_CALL "VK7IAN"

#if SI5351_PLL_VCO_MIN != 440000000
#err set SI5351_PLL_VCO_MIN to 440000000 in si5351.h
#endif

#define VERSION_STRING "  V2.4."
#define CW_TIMEOUT 800u
#define MENU_TIMEOUT 5000u
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
#define DEFAULT_BANDWIDTH 3ul
#define BUTTON_LONG_PRESS_TIME 800ul
#define TCXO_FREQ 27000021ul
#define SPECTRUM_LEVEL_MIN -4
#define SPECTRUM_LEVEL_MAX 4
#define LPF_I2C_ADDRESS 0x20U
#define BPF_I2C_ADDRESS 0x21U

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
#define POS_SPLASH_X       80
#define POS_SPLASH_Y       50
#define POS_VERSION_X       0
#define POS_VERSION_Y     122
#define POS_FREQUENCY_X    60
#define POS_FREQUENCY_Y     0
#define POS_TX_X            0
#define POS_TX_Y            5
#define POS_RX_X           30
#define POS_RX_Y            5
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
#define WATERFALL_ROWS 41

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
  uint32_t spectype;
  uint32_t jnrlevel;
  uint32_t micgain;
  uint32_t bandwidth;
  radio_mode_t mode;
  bool tx_button;
  bool tx_enable;
  bool keydown;
  bool cessb;
  bool gaussian;
  bool menu_active;
  bool mode_auto;
  bool graph_swr;
  int8_t level[NUM_BANDS];
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
  SPECTRUM_WIND,
  JNR_OFF,
  DEFAULT_MICGAIN,
  DEFAULT_BANDWIDTH,
  DEFAULT_MODE,
  false,
  false,
  false,
  false,
  true,
  false,
  true,
  false,
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
volatile static uint32_t mic_peak_level = 0;
volatile static int16_t adc_data_i[MAX_ADC_SAMPLES] = {0};
volatile static int16_t adc_data_q[MAX_ADC_SAMPLES] = {0};
volatile static float mic_gain = 0.0f;
volatile static bool dit_latched = false;
volatile static bool dah_latched = false;
volatile static bool save_settings_now = false;
volatile static bool setup_complete = false;
volatile static bool set_spectrum_level = false;

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

static void display_clear(void)
{
  lcd.fillSprite(LCD_BLACK);
}

static void display_refresh(void)
{
  lcd.pushSprite(0,0);
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

static void set_frequency(void)
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
    case MODE_AM:  rx_ssboffset = -(SAMPLERATE/4); break;
    case MODE_CWL: rx_cwoffsetw = +cwoffset;       break;
    case MODE_CWU: rx_cwoffsetw = -cwoffset;       break;
  }
  const uint64_t f = SI5351_FREQ_MULT * (radio.frequency+rx_ssboffset+rx_cwoffsetw);
  const uint64_t p = SI5351_FREQ_MULT * (radio.frequency+rx_ssboffset+rx_cwoffsetw)*quadrature_divisor;
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
  pinMode(PIN_MIC,OUTPUT);
  digitalWrite(PIN_MIC,LOW);

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
    lcd.setTextSize(3);
    lcd.setTextColor(spectrum::color_map_32[i],LCD_BLACK);
    lcd.setCursor(POS_SPLASH_X,POS_SPLASH_Y);
    lcd.print("MBPTRX");
    lcd.setTextSize(1);
    lcd.setTextColor(LCD_WHITE,LCD_BLACK);
    lcd.setCursor(POS_VERSION_X,POS_VERSION_Y);
    lcd.print(sz_version);
    lcd.pushSprite(0,0);
    delay(50);
  }
  lcd.fillSprite(LCD_BLACK);
  lcd.pushSprite(0,0);

  // intro screen
  lcd.fillSprite(LCD_BLACK);
  lcd.setTextColor(LCD_WHITE,LCD_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(210,120);
  lcd.print(sz_version);
  lcd.setTextSize(3);
  lcd.setCursor(80,40);
#ifdef YOUR_CALL
  lcd.print(YOUR_CALL);
#else
  lcd.print("VK7IAN");
#endif
  lcd.pushSprite(0,0);

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
  const uint16_t fcolour = radio.tx_enable?LCD_RED:LCD_WHITE;
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
  lcd.setTextSize(3);
  lcd.setTextColor(fcolour,LCD_BLACK);
  lcd.setCursor(POS_FREQUENCY_X,POS_FREQUENCY_Y);
  lcd.print(sz_frequency);
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
  lcd.setTextSize(2);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(LCD_BLUE);
  lcd.print("RX");
}

static void show_rx(void)
{
  lcd.setTextSize(2);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(LCD_BLUE);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(LCD_WHITE);
  lcd.print("RX");
}

static void show_swl(void)
{
  lcd.setTextSize(2);
  lcd.setCursor(POS_TX_X,POS_TX_Y);
  lcd.setTextColor(radio.tx_button?LCD_RED:LCD_GREEN);
  lcd.print("TX");
  lcd.setCursor(POS_RX_X,POS_RX_Y);
  lcd.setTextColor(radio.tx_button?LCD_RED:LCD_GREEN);
  lcd.print("RX");
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
    // TX in AM is always narrow
    switch (radio.mode)
    {
      case MODE_LSB: sz_mode = "LSB"; break;
      case MODE_USB: sz_mode = "USB"; break;
      case MODE_CWL: sz_mode = "CWL"; break;
      case MODE_CWU: sz_mode = "CWU"; break;
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
  if (set_spectrum_level)
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
  }
}

static void show_cessb_settings(void)
{
  if (set_spectrum_level)
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
    if (radio.mode==MODE_LSB || radio.mode==MODE_USB)
    {
      lcd.setTextSize(1);
      lcd.setTextColor(LCD_GREEN);
      lcd.setCursor(POS_CESSB_MIC_X,POS_CESSB_MIC_Y);
      lcd.print("CESSB:");
      lcd.print(radio.cessb?"On":"Off");
      lcd.print("  Mic:");
      lcd.print(radio.micgain);
      lcd.print("%");
      lcd.print("  BW:");
      switch (radio.bandwidth)
      {
        case 1: lcd.print("2000Hz"); break;
        case 2: lcd.print("2200Hz"); break;
        case 3: lcd.print("2400Hz"); break;
        case 4: lcd.print("2600Hz"); break;
        case 5: lcd.print("2800Hz"); break;
      }
    }
  }
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
  lcd.setTextSize(2);
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
      lcd.setCursor(POS_MENU_X+4,POS_MENU_Y+20*j+4);
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
        lcd.setCursor(POS_MENU_X+4,POS_MENU_Y+2);
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
          lcd.setCursor(POS_MENU_X+4,POS_MENU_Y+20*j+2);
          lcd.print(sz_option_name);
        }
      }
    }
  }
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
        int16_t tx_i = 0;
        int16_t tx_q = 0;
        switch (radio.mode)
        {
          case MODE_LSB: DSP::process_mic(adc_value,tx_q,tx_i,mic_gain,radio.cessb); break;
          case MODE_USB: DSP::process_mic(adc_value,tx_i,tx_q,mic_gain,radio.cessb); break;
          case MODE_CWL: CW::process_cw(radio.keydown,tx_i,tx_q);                    break;
          case MODE_CWU: CW::process_cw(radio.keydown,tx_i,tx_q);                    break;
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
        adc_sample_p++;
        adc_sample_p &= (MAX_ADC_SAMPLES-1);
        if (radio.mode==MODE_LSB || radio.mode==MODE_USB)
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
        volatile const uint32_t cpu_end = time_us_32();
        cpu.usage += (cpu_end-cpu_start);
        cpu.count++;
      }
    }
    else
    {
      // switched to RX
      pinMode(PIN_MIC,OUTPUT);
      digitalWrite(PIN_MIC,LOW);
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
      adc_gpio_init(PIN_MIC);
      adc_select_input(MIC_MUX);
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
        if (i2s.available())
        {
          int32_t ii = 0;
          int32_t qq = 0;
          i2s.read32(&ii, &qq);
          ii >>= 16;
          qq >>= 16;
          adc_data_i[adc_sample_p] = (int16_t)ii;
          adc_data_q[adc_sample_p] = (int16_t)qq;
          adc_sample_p++;
          adc_sample_p &= (MAX_ADC_SAMPLES-1);
          int32_t dac_audio = 0;
          const uint8_t jnr = radio.jnrlevel;
          const uint8_t bw = radio.bandwidth - 1;
          switch (radio.mode)
          {
            case MODE_LSB: dac_audio = DSP::process_ssb(qq,ii,jnr,bw); break;
            case MODE_USB: dac_audio = DSP::process_ssb(ii,qq,jnr,bw); break;
            case MODE_AM:  dac_audio = DSP::process_am(ii,qq,jnr);     break;
            case MODE_CWL: dac_audio = DSP::process_cw(qq,ii);         break;
            case MODE_CWU: dac_audio = DSP::process_cw(ii,qq);         break;
          }
          dac_audio = constrain(dac_audio,-2048l,+2047l);
          dac_audio += 2048l;
          dac_h = dac_audio >> 6;
          dac_l = dac_audio & 0x3f;
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
  int16_t data_re[1024] = {0};
  int16_t data_im[1024] = {0};
  const int8_t gain = radio.tx_enable?0:radio.level[radio.band];
  const uint32_t sample_p = adc_sample_p;
  if (sample_p<800)
  {
    // first half is in use, get the second half
    for (uint32_t i=0;i<1024;i++)
    {
      data_re[i] = adc_data_i[i+1024];
      data_im[i] = adc_data_q[i+1024];
    }
    spectrum::process(data_re,data_im,magnitude,gain);
  }
  else if (sample_p>1023 && sample_p<1800)
  {
    // second half is in use, get the first half
    for (uint32_t i=0;i<1024;i++)
    {
      data_re[i] = adc_data_i[i];
      data_im[i] = adc_data_q[i];
    }
    spectrum::process(data_re,data_im,magnitude,gain);
  }
}

static void process_ssb_tx(void)
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

void loop1(void)
{
  // run UI on core 1
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
        case OPTION_MODE_LSB:       radio.mode = MODE_LSB; radio.mode_auto = false; break;
        case OPTION_MODE_USB:       radio.mode = MODE_USB; radio.mode_auto = false; break;
        case OPTION_MODE_CWL:       radio.mode = MODE_CWL; radio.mode_auto = false; break;
        case OPTION_MODE_CWU:       radio.mode = MODE_CWU; radio.mode_auto = false; break;
        case OPTION_MODE_AM:        radio.mode = MODE_AM;  radio.mode_auto = false; break;
        case OPTION_MODE_AUTO:      radio.mode_auto = true;                         break;
        case OPTION_STEP_10:        radio.step = 10U;                               break;
        case OPTION_STEP_100:       radio.step = 100U;                              break;
        case OPTION_STEP_500:       radio.step = 500U;                              break;
        case OPTION_STEP_1000:      radio.step = 1000U;                             break;
        case OPTION_STEP_5000:      radio.step = 5000U;                             break;
        case OPTION_STEP_10000:     radio.step = 10000U;                            break;
        case OPTION_STEP_100000:    radio.step = 100000U;                           break;
        case OPTION_BAND_80M:       radio.band = BAND_80M;                          break;
        case OPTION_BAND_40M:       radio.band = BAND_40M;                          break;
        case OPTION_BAND_30M:       radio.band = BAND_30M;                          break;
        case OPTION_BAND_20M:       radio.band = BAND_20M;                          break;
        case OPTION_BAND_17M:       radio.band = BAND_17M;                          break;
        case OPTION_BAND_15M:       radio.band = BAND_15M;                          break;
        case OPTION_BAND_12M:       radio.band = BAND_12M;                          break;
        case OPTION_BAND_10M:       radio.band = BAND_10M;                          break;
        case OPTION_BAND_SWL:       radio.band = BAND_SWL;                          break;
        case OPTION_BW_2000:        radio.bandwidth = 1ul;                          break;
        case OPTION_BW_2200:        radio.bandwidth = 2ul;                          break;
        case OPTION_BW_2400:        radio.bandwidth = 3ul;                          break;
        case OPTION_BW_2600:        radio.bandwidth = 4ul;                          break;
        case OPTION_BW_2800:        radio.bandwidth = 5ul;                          break;
        case OPTION_SIDETONE_500:   radio.sidetone = 500u;                          break;
        case OPTION_SIDETONE_550:   radio.sidetone = 550u;                          break;
        case OPTION_SIDETONE_600:   radio.sidetone = 600u;                          break;
        case OPTION_SIDETONE_650:   radio.sidetone = 650u;                          break;
        case OPTION_SIDETONE_700:   radio.sidetone = 700u;                          break;
        case OPTION_SIDETONE_750:   radio.sidetone = 750u;                          break;
        case OPTION_SIDETONE_800:   radio.sidetone = 800u;                          break;
        case OPTION_SIDETONE_850:   radio.sidetone = 850u;                          break;
        case OPTION_SIDETONE_LOW:   radio.cw_level = 1u;                            break;
        case OPTION_SIDETONE_MED:   radio.cw_level = 2u;                            break;
        case OPTION_SIDETONE_HI:    radio.cw_level = 3u;                            break;
        case OPTION_CW_SPEED_10:    radio.cw_dit = 120u;                            break;
        case OPTION_CW_SPEED_15:    radio.cw_dit = 80u;                             break;
        case OPTION_CW_SPEED_20:    radio.cw_dit = 60u;                             break;
        case OPTION_CW_SPEED_25:    radio.cw_dit = 48u;                             break;
        case OPTION_CW_SPEED_30:    radio.cw_dit = 40u;                             break;
        case OPTION_SPECTRUM_WIND:  radio.spectype = SPECTRUM_WIND;                 break;
        case OPTION_SPECTRUM_GRASS: radio.spectype = SPECTRUM_GRASS;                break;
        case OPTION_SPECTRUM_LEVEL: set_spectrum_level = true;                      break;
        case OPTION_JNR_LEVEL1:     radio.jnrlevel = JNR_LEVEL1;                    break;
        case OPTION_JNR_LEVEL2:     radio.jnrlevel = JNR_LEVEL2;                    break;
        case OPTION_JNR_LEVEL3:     radio.jnrlevel = JNR_LEVEL3;                    break;
        case OPTION_JNR_OFF:        radio.jnrlevel = JNR_OFF;                       break;
        case OPTION_MIC_25:         radio.micgain = 25;                             break;
        case OPTION_MIC_50:         radio.micgain = 50;                             break;
        case OPTION_MIC_75:         radio.micgain = 75;                             break;
        case OPTION_MIC_100:        radio.micgain = 100;                            break;
        case OPTION_MIC_125:        radio.micgain = 125;                            break;
        case OPTION_MIC_150:        radio.micgain = 150;                            break;
        case OPTION_MIC_175:        radio.micgain = 175;                            break;
        case OPTION_MIC_200:        radio.micgain = 200;                            break;
        case OPTION_CESSB_ON:       radio.cessb = true;                             break;
        case OPTION_CESSB_OFF:      radio.cessb = false;                            break;
        case OPTION_GAUSSIAN_ON:    radio.gaussian = true;                          break;
        case OPTION_GAUSSIAN_OFF:   radio.gaussian = false;                         break;
        case OPTION_GRAPH_SWR_Y:    radio.graph_swr = true;                         break;
        case OPTION_GRAPH_SWR_N:    radio.graph_swr = false;                        break;
        case OPTION_EXIT:           radio.menu_active = false;                      break;
      }

      if (set_spectrum_level)
      {
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
      if (radio.mode!=old_mode)
      {
        old_mode = radio.mode;
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
      if (set_spectrum_level)
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
          if (new_level!=old_level)
          {
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
  process_spectrum();

  // update the display every 50ms
  static uint32_t next_update = 0;
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update = now + 50ul;
    update_display(DSP::smeter(radio.frequency));
  }

  // check for PTT
  const bool b_PTT = (digitalRead(PIN_PTT)==LOW);
  const bool b_PADA = (digitalRead(PIN_PADA)==LOW);
  const bool b_PADB = (digitalRead(PIN_PADB)==LOW);
  radio.tx_button = (b_PTT || b_PADA || b_PADB);
  if ((radio.band == BAND_SWL) || (radio.mode == MODE_AM))
  {
    return;
  }
  if (radio.tx_button)
  {
    const float saved_agc = DSP::agc_peak;
    radio.menu_active = false;
    if (set_spectrum_level)
    {
      // restore old level in case it was changed
      set_spectrum_level = false;
      radio.level[radio.band] = old_level;
    }
    if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
    {
      process_key();
    }
    else if (b_PTT)
    {
      process_ssb_tx();
    }

    // back to receive
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