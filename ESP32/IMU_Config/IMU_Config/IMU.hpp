#ifndef IMU_HPP
#define IMU_HPP

#include "I2C_Abstract.hpp"
#include "stdbool.h"

typedef struct FifoSample {
  uint8_t tag;  // FIFO tag byte
  int16_t x;
  int16_t y;
  int16_t z;
} fifoSample_t;

typedef struct {
  bool have_accel;
  bool have_gyro;

  int16_t ax_last, ay_last, az_last;
  int16_t gx_last, gy_last, gz_last;
} FifoDecompState;

struct CombinedSample {
  bool hasAccel = false;
  bool hasGyro = false;

  float ax, ay, az;
  float gx, gy, gz;
};

struct Calibration {
  float ax_off = 0;
  float ay_off = 0;
  float az_off = 0;

  float gx_off = 0;
  float gy_off = 0;
  float gz_off = 0;
};

bool isGyroData(uint8_t rawTag);
bool isAccelData(uint8_t rawTag);


size_t dumpFirstNTags(uint16_t n, uint32_t timeout_ms = 1000);
bool readUI(uint8_t reg, uint8_t &val);
bool readEmbedded(uint8_t reg, uint8_t &val);
bool writeEmbedded(uint8_t reg, uint8_t val);
bool shubWrite(uint8_t reg, uint8_t val);
bool shubRead(uint8_t reg, uint8_t &val);
void dumpSHUBConfigs();
void dumpIMURegisters_Fixed();
bool setFuncCfg(bool enable);
void disableFifoCompression();
static int16_t signExtend5(int v);
size_t decodeFifoWord(uint8_t rawTag,
                      const uint8_t data[6],
                      fifoSample_t *out,
                      size_t maxOut); 
size_t readFIFO(fifoSample_t *buffer,  FifoDecompState &decomp, size_t maxSamples = 300);
bool enableAccelGyro(uint16_t frequencyHz);
void resetAndBypass();
void calibrateIMU(struct Calibration &calib, FifoDecompState &decomp, size_t samples = 300);

#endif
