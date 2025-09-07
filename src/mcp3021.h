#ifndef MCP3021_H
#define MCP3021_H

#include "Wire.h"

class MCP3021
{
  public:
    const bool begin(const uint8_t deviceId);
  	const bool begin(void);
    const uint16_t read(void);
			
  private:		
    static const uint8_t baseAddress = 0b01001000;
    uint8_t _deviceAddress;
};

// device at specified address
const bool MCP3021::begin(const uint8_t deviceId)
{
  _deviceAddress = (baseAddress|deviceId);
#ifdef DEBUGGING_SKIP
  return true;
#else
  Wire.beginTransmission(_deviceAddress);
  return (Wire.endTransmission() == 0);
#endif
}

// default device at default address
const bool MCP3021::begin(void)
{
  _deviceAddress = baseAddress|0b00000101;
#ifdef DEBUGGING_SKIP
  return true;
#else
  Wire.beginTransmission(_deviceAddress);
  return (Wire.endTransmission() == 0);
#endif
}

const uint16_t MCP3021::read(void)
{
  // request 2 bytes from MCP3021
#ifdef DEBUGGING_SKIP
  return 0u;
#else
  Wire.requestFrom(_deviceAddress, 2);
  if (Wire.available()==2)
  {
    // MSB bits in lower 4 bits of first byte
    // LSB bits in upper 6 bits of second byte
    return ((Wire.read()<<6) | (Wire.read()>>2));
  }
  return 0xffffu;
#endif
}

#endif