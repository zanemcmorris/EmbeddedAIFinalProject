#include "I2C_Abstract.hpp"

uint8_t imuAddr = LSM6DSOX_ADDR_6A;


// ---- helpers for raw I2C register access ----
bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool read8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, 1) != 1) return false;
  val = Wire.read();
  return true;
}

bool readBytes(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)imuAddr, (int)len);
  for (size_t i = 0; i < got && i < len; i++) buf[i] = Wire.read();
  return got == len;
}