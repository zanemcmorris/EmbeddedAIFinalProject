#ifndef IMU_HPP
#define IMU_HPP

#include "I2C_Abstract.hpp"
#include "stdbool.h"

// --- Tag bitfield helpers (from ST code) ---
#define TAG_COUNTER_MASK           (0x06u)
#define TAG_SENSOR_MASK            (0xF8u)
#define TAG_COUNTER_SHIFT          (0x01u)
#define TAG_SENSOR_SHIFT           (0x03u)

// Tags (only the ones we actually care about)
#define TAG_EMPTY                  (0x00u)
#define TAG_GY                     (0x01u)
#define TAG_XL                     (0x02u)
#define TAG_TEMP                   (0x03u)
#define TAG_TS                     (0x04u)
#define TAG_ODRCHG                 (0x05u)
#define TAG_XL_UNCOMPRESSED_T_2    (0x06u)
#define TAG_XL_UNCOMPRESSED_T_1    (0x07u)
#define TAG_XL_COMPRESSED_2X       (0x08u)
#define TAG_XL_COMPRESSED_3X       (0x09u)
#define TAG_GY_UNCOMPRESSED_T_2    (0x0Au)
#define TAG_GY_UNCOMPRESSED_T_1    (0x0Bu)
#define TAG_GY_COMPRESSED_2X       (0x0Cu)
#define TAG_GY_COMPRESSED_3X       (0x0Du)

enum SensorType {
  SENSOR_NONE = 0,
  SENSOR_ACCEL,
  SENSOR_GYRO
};

enum CompressionType {
  COMP_NC,      // normal (uncompressed or T0)
  COMP_NC_T1,   // uncompressed, T-1
  COMP_NC_T2,   // uncompressed, T-2
  COMP_2X,      // compressed 2x
  COMP_3X       // compressed 3x
};

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
size_t readFIFO(fifoSample_t *buffer, FifoDecompState &decomp, size_t maxSamples = 300);
bool enableAccelGyro(uint16_t frequencyHz);
void resetAndBypass();
void calibrateIMU(struct Calibration &calib, FifoDecompState &decomp, size_t samples = 300);

void readAccelDirect(int16_t &ax, int16_t &ay, int16_t &az);

#endif
