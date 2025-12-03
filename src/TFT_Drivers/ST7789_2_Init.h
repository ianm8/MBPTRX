
// This is the command sequence that initialises the ST7789 driver

// Configure ST7789 display
{
  static const uint8_t PROGMEM
  st7789[] = {
    //8,
    7,
    TFT_SLPOUT, TFT_INIT_DELAY, 255,
    TFT_COLMOD, 1+TFT_INIT_DELAY, 0x55, 10,
    TFT_MADCTL, 1, 0x00,
    TFT_CASET, 4, 0x00, 0x00, 0x00, 0xF0,
    TFT_PASET, 4, 0x00, 0x00, 0x00, 0xF0,
    TFT_INVON, TFT_INIT_DELAY, 10,
    TFT_NORON, TFT_INIT_DELAY, 10,
    TFT_DISPON, TFT_INIT_DELAY, 255
    };

  commandList(st7789);
  //fillScreen(TFT_BLACK);
  //writecommand(ST7789_DISPON);    //Display on
  //writecommand(TFT_DISPON);    //Display on
 
  //delay(500);
}
// End of ST7789 display configuration