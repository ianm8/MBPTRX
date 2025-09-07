/*
 * Copyright (c) 2020 Hideaki Tai
 * 
 * modified Ian Mitchell 2025
 * 
 */

#pragma once
#ifndef ARDUINO_TCA9534A_H
#define ARDUINO_TCA9534A_H

#include <Arduino.h>
#include <Wire.h>

namespace arduino
{
  template <typename WireType>
  class IoEx8bit
  {
    uint8_t I2C_ADDR = 0x20;

    WireType* wire=NULL;
    volatile uint8_t sts;

  public:

    enum class Reg { INPUT_PORT, OUTPUT_PORT, POLARITY, CONFIG };
    enum class Config { OUT, IN };
    enum class Polarity { ORIGINAL, INVERSE };
    enum class Level { L, H };

    void attach(WireType& w) { wire = &w; }

    const bool setDeviceAddress(const uint8_t addr)
    {
      if (wire==NULL)
      {
        return false;
      }
      I2C_ADDR = addr;
#ifdef DEBUGGING_SKIP
      return true;
#endif
      wire->beginTransmission(I2C_ADDR);
      return (wire->endTransmission() == 0);
    }

    uint8_t input(const uint8_t port)
    {
      return readBit(I2C_ADDR, (uint8_t)Reg::INPUT_PORT, port);
    }

    uint8_t input()
    {
      return readByte(I2C_ADDR, (uint8_t)Reg::INPUT_PORT);
    }

    uint8_t output(const uint8_t port, const Level v)
    {
      return writeBit(I2C_ADDR, (uint8_t)Reg::OUTPUT_PORT, port, (uint8_t)v);
    }

    uint8_t output(const Level v)
    {
      uint8_t d = (v == Level::L) ? 0x00 : 0xFF;
      return writeByte(I2C_ADDR, (uint8_t)Reg::OUTPUT_PORT, (uint8_t)d);
    }

    uint8_t output()
    {
      return readByte(I2C_ADDR, (uint8_t)Reg::OUTPUT_PORT);
    }

    uint8_t polarity(const uint8_t port, const Polarity pol)
    {
      return writeBit(I2C_ADDR, (uint8_t)Reg::POLARITY, port, (uint8_t)pol);
    }

    uint8_t polarity(const Polarity pol)
    {
      uint8_t d = (pol == Polarity::ORIGINAL) ? 0x00 : 0xFF;
      return writeByte(I2C_ADDR, (uint8_t)Reg::POLARITY, (uint8_t)d);
    }

    uint8_t polarity()
    {
      return readByte(I2C_ADDR, (uint8_t)Reg::POLARITY);
    }

    uint8_t config(const uint8_t port, const Config cfg)
    {
      return writeBit(I2C_ADDR, (uint8_t)Reg::CONFIG, port, (uint8_t)cfg);
    }

    uint8_t config(const Config cfg)
    {
      uint8_t d = (cfg == Config::OUT) ? 0x00 : 0xFF;
      return writeByte(I2C_ADDR, (uint8_t)Reg::CONFIG, (uint8_t)d);
    }

    uint8_t config()
    {
      return readByte(I2C_ADDR, (uint8_t)Reg::CONFIG);
    }

    uint8_t status() const { return sts; }

  private:

    uint8_t readBit(uint8_t dev, uint8_t reg, uint8_t bit)
    {
      uint8_t b = readByte(dev, reg);
      b &= (1 << bit);
      return b;
    }

    uint8_t readByte(uint8_t dev, uint8_t reg)
    {
      uint8_t data = 0;
#ifndef DEBUGGING_SKIP
      if (wire!=NULL)
      {
        wire->beginTransmission(dev);
        wire->write(reg);
        wire->endTransmission();
        wire->requestFrom(dev, 1);
        if (wire->available()) data = wire->read();
      }
#endif
      return data;
    }

    bool writeBit(uint8_t dev, uint8_t reg, uint8_t bit, uint8_t data)
    {
      uint8_t b = readByte(dev, reg);
      b = (data != 0) ? (b | (1 << bit)) : (b & ~(1 << bit));
      return writeByte(dev, reg, b);
    }

    bool writeByte(uint8_t dev, uint8_t reg, uint8_t data)
    {
      if (wire==NULL)
      {
        return false;
      }
#ifdef DEBUGGING_SKIP
      return true;
#else
      wire->beginTransmission(dev);
      wire->write(reg);
      wire->write(data);
      sts = wire->endTransmission();
      return (sts == 0);
#endif
    }
  };
}

using TCA9534 = arduino::IoEx8bit<TwoWire>;

#endif // ARDUINO_TCA9534A_H
