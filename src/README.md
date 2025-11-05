Libraries
 *  https://github.com/Bodmer/TFT_eSPI

Important:
Copy "User_Setup.h" to  ..\Arduino\libraries\TFT_eSPI whenever the TFT_eSPI library is installed. Important note, there is a bug in version that requires the following change in TFT_eSPI_RP2040.h:

     #define SET_BUS_READ_MODE  // spi_set_format(SPI_X,  8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST)

   that is, comment out: spi_set_format(SPI_X,  8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST)
 
Build:
 *  Pi Pico 2
 *  CPU Speed: 240Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 4MB (no FS)

Some history
 * 0.1.240 start with uSBITX and iqSDR40 code
 * 0.2.240 fix read I2C port bug
 * 0.3.240 fix RX/TX relay bug
 * 0.4.240 spectrum level
 * 0.5.240 direct CW
 * 0.6.240 complex mixer offset
 * 0.7.240 update bandwidths
 * 0.8.240 enable EEPROM writes
 * 0.9.240 fix CW display width
 * 1.0.240 RX bandwith options
 * 1.1.240 code cleanup
 * 1.2.240 fix SWR display
 * 1.3.240 graph power and SWR
 * 1.4.240 set spectrum level
 * 1.5.240 exit level if TX
