## Libraries
 *  ~~https://github.com/Bodmer/TFT_eSPI~~
 *  A modified version of this library is now included
 *  Install the TFT_eSPI2 library provided in the **release** files
 *  This version adds a **displayOn()** function so that the display can be initialised and cleared before being switched on
 
## Build
 *  Pi Pico 2
 *  CPU Speed: 240Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 4MB (no FS)

## Some history
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
 * 1.6.240 smeter sensitivity higher bands
 * 1.7.240 change TX frequency colour
 * 1.8.240 fix CW display position
 * 2.0.240 AM mode for SWL band
 * 2.1.240 improved AM demodulation
 * 2.2.240 include modified TFT_eSPI library
 * 2.3.240 fix TFT display initialisation
 * 2.4.240 separate modified TFT_eSPI2 library
 * 2.5.240 add exit option to JNR menu

