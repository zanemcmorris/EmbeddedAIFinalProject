#ifndef I2C_ABSTRACT_HPP
#define I2C_ABSTRACT_HPP

#include <Wire.h>
#include <Arduino.h>

// I2C addresses depend on SDO wiring
#define LSM6DSOX_ADDR_6A 0x6A
#define LSM6DSOX_ADDR_6B 0x6B

// Registers
#define REG_CTRL3_C 0x12
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79  // up to 0x7E
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_EMB_FUNC_EN_A 0x04
#define REG_EMB_FUNC_EN_B 0x05
#define SHUB_ACCESS 0x40      // FUNC_CFG_ACCESS.SHUB_REG_ACCESS
#define REG_SLV0_CONFIG 0x17  // SHUB page
#define REG_SLV1_CONFIG 0x19
#define REG_SLV2_CONFIG 0x1B
#define REG_SLV3_CONFIG 0x1D

#define REG_OUTX_L_A  0x28
#define REG_OUTX_H_A  0x29
#define REG_OUTY_L_A  0x2A
#define REG_OUTY_H_A  0x2B
#define REG_OUTZ_L_A  0x2C
#define REG_OUTZ_H_A  0x2D


// ---- helpers for raw I2C register access ----
bool write8(uint8_t reg, uint8_t val);
bool read8(uint8_t reg, uint8_t &val);
bool readBytes(uint8_t reg, uint8_t *buf, size_t len);


#endif