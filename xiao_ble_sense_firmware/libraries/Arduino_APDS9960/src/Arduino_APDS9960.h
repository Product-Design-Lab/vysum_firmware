/*
  This file is part of the Arduino_APDS9960 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef ARDUINO_APDS9960
#define ARDUINO_APDS9960

#include <Arduino.h>
#include <Wire.h>

enum {
  GESTURE_NONE = -1,
  GESTURE_UP = 0,
  GESTURE_DOWN = 1,
  GESTURE_LEFT = 2,
  GESTURE_RIGHT = 3
};

class APDS9960 {
public:
  APDS9960(TwoWire &wire, int intPin);
  virtual ~APDS9960();

  bool begin();
  void end();

  int gestureAvailable();
  int readGesture();

  int colorAvailable();
  bool readColor(int& r, int& g, int& b);
  bool readColor(int& r, int& g, int& b, int& c);

  int proximityAvailable();
  int readProximity();

  void setGestureSensitivity(uint8_t sensitivity);

  void setInterruptPin(int pin);

  bool setLEDBoost(uint8_t boost);

private:
  bool setGestureIntEnable(bool en);
  bool setGestureMode(bool en);
  int gestureFIFOAvailable();
  int handleGesture();

  bool enablePower();
  bool disablePower();
  bool enableColor();
  bool disableColor();
  bool enableProximity();
  bool disableProximity();
  bool enableWait();
  bool disableWait();
  bool enableGesture();
  bool disableGesture();

private:
  TwoWire& _wire;
  int _intPin;

  bool _gestureEnabled;
  bool _proximityEnabled;
  bool _colorEnabled;
  bool _gestureIn;
  int _gestureDirectionX;
  int _gestureDirectionY;
  int _gestureDirInX;
  int _gestureDirInY;
  int _gestureSensitivity;
  int _detectedGesture;

  bool write(uint8_t val);
  bool write(uint8_t reg, uint8_t val);
  bool read(uint8_t reg, uint8_t *val);
  size_t readBlock(uint8_t reg, uint8_t *val, unsigned int len);

private:
#define REG(name, addr) \
  bool get##name(uint8_t *val) { return read(addr,  val); } \
  bool set##name(uint8_t val)  { return write(addr, val); } \
  size_t read##name(uint8_t *val, uint8_t len) { return readBlock(addr, val, len); }
  REG(ENABLE,     0x80) // The ENABLE register is used to power the device on/off, enable functions and interrupts.
  REG(ATIME,      0x81) // ADC Integration Time Register controls the internal integration time of ALS/Color analog to digital converters
  REG(WTIME,      0x83) // Wait Time Register controls the amount of time in a low power mode between Proximity and/or ALS cycles
  REG(AILTL,      0x84) // ALS Interrupt Threshold Register
  REG(AILTH,      0x85)
  REG(AIHTL,      0x86)
  REG(AIHTH,      0x87)
  REG(PILT,       0x89) // Proximity Interrupt Threshold Register
  REG(PIHT,       0x8B)
  REG(PERS,       0x8C) // Persistence Register
  REG(CONFIG1,    0x8D) // The CONFIG1 register sets the wait long time. The register is set to 0x40 at power up.
  /*
  Proximity Pulse Count Register (0x8E)
  The Proximity Pulse Count Register sets Pulse Width Modified current during a Proximity Pulse. The proximity pulse
  count register bits set the number of pulses to be output on the LDR pin. The Proximity Length register bits set the
  amount of time the LDR pin is sinking current during a proximity pulse.
  */
  REG(PPULSE,     0x8E)
  REG(CONTROL,    0x8F)
  REG(CONFIG2,    0x90)
  REG(ID,         0x92)
  REG(STATUS,     0x93)
  REG(CDATAL,     0x94)
  REG(CDATAH,     0x95)
  REG(RDATAL,     0x96)
  REG(RDATAH,     0x97)
  REG(GDATAL,     0x98)
  REG(GDATAH,     0x99)
  REG(BDATAL,     0x9A)
  REG(BDATAH,     0x9B)
  REG(PDATA,      0x9C)
  REG(POFFSET_UR, 0x9D)
  REG(POFFSET_DL, 0x9E)
  REG(CONFIG3,    0x9F)
  REG(GPENTH,     0xA0)
  REG(GEXTH,      0xA1)
  REG(GCONF1,     0xA2)
  REG(GCONF2,     0xA3)
  REG(GOFFSET_U,  0xA4)
  REG(GOFFSET_D,  0xA5)
  REG(GPULSE,     0xA6)
  REG(GOFFSET_L,  0xA7)
  REG(GOFFSET_R,  0xA9)
  REG(GCONF3,     0xAA)
  REG(GCONF4,     0xAB)
  REG(GFLVL,      0xAE)
  REG(GSTATUS,    0xAF)
  REG(IFORCE,     0xE4)
  REG(PICLEAR,    0xE5)
  REG(CICLEAR,    0xE6)
  REG(AICLEAR,    0xE7)
  REG(GFIFO_U,    0xFC)
  REG(GFIFO_D,    0xFD)
  REG(GFIFO_L,    0xFE)
  REG(GFIFO_R,    0xFF)
};

extern APDS9960 APDS;

#endif // ARDUINO_APDS9960
