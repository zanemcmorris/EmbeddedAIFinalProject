#include <Wire.h>
#include "IMU.hpp"

const float ACC_SENS_16G      = 0.488f;   // mg/LSB
const float GYRO_SENS_500DPS  = 17.50f;   // mdps/LSB

static SensorType     getSensorTypeFromTag(uint8_t tag);
static CompressionType getCompressionTypeFromTag(uint8_t tag);
static void get_diff_2x(int16_t diff[6], const uint8_t input[6]);
static void get_diff_3x(int16_t diff[9], const uint8_t input[6]);

bool isCalibrated = false;

FifoDecompState decomp = { 0 };
Calibration     calib;

// ---------------------------------------------------------------------------
// IMU STARTUP
// ---------------------------------------------------------------------------
bool startIMU(uint16_t sensorFreq) {
  bool status = true;

  if (!Serial) {
    Serial.begin(115200);
    delay(200);
  }

  // tinyCore: IMU power / enable pin
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);   // turn IMU power on
  delay(5);

  // I2C on tinyCore IMU pins: SDA=3, SCL=4
  if (!Wire.begin(3, 4, 400000)) {
    Serial.println("Could not start I2C Wire...");
    status = false;
    return status;
  }

  resetAndBypass();

  write8(REG_CTRL3_C, (1 << 6) | (1 << 2));  // Ensure auto-increment + block data update
  write8(0x19 /*CTRL10_C*/, 0x00);           // Disable timestamps
  shubWrite(REG_SLV0_CONFIG, 0x00);
  shubWrite(REG_SLV1_CONFIG, 0x00);
  shubWrite(REG_SLV2_CONFIG, 0x00);
  shubWrite(REG_SLV3_CONFIG, 0x00);
  disableFifoCompression();

  if (!enableAccelGyro(sensorFreq)) {
    Serial.println("Could not set sensor freq and scale");
    while (1) { delay(10); }
  }

  write8(REG_FIFO_CTRL3, (0b0100 << 4) | (0b0100));  // BDR_XL=104 Hz, BDR_G=104 Hz
  write8(REG_FIFO_CTRL4, 0x00);                      // Clear all old data
  write8(REG_FIFO_CTRL4, 0x06);                      // Continuous mode, overwrite old data

  dumpIMURegisters_Fixed();
  calibrateIMU(300);

  return status;
}

// ---------------------------------------------------------------------------
// TAG HELPERS
// ---------------------------------------------------------------------------
bool isGyroData(uint8_t rawTag) {
  uint8_t tag = (rawTag & TAG_SENSOR_MASK) >> TAG_SENSOR_SHIFT;
  switch (tag) {
    case TAG_GY:
    case TAG_GY_UNCOMPRESSED_T_2:
    case TAG_GY_UNCOMPRESSED_T_1:
    case TAG_GY_COMPRESSED_2X:
    case TAG_GY_COMPRESSED_3X:
      return true;
    default:
      return false;
  }
}

bool isAccelData(uint8_t rawTag) {
  uint8_t tag = (rawTag & TAG_SENSOR_MASK) >> TAG_SENSOR_SHIFT;
  switch (tag) {
    case TAG_XL:
    case TAG_XL_UNCOMPRESSED_T_2:
    case TAG_XL_UNCOMPRESSED_T_1:
    case TAG_XL_COMPRESSED_2X:
    case TAG_XL_COMPRESSED_3X:
      return true;
    default:
      return false;
  }
}

bool isGyroData(fifoSample_t *sample) {
  return isGyroData(sample->tag);
}

bool isAccelData(fifoSample_t *sample) {
  return isAccelData(sample->tag);
}

// ---------------------------------------------------------------------------
// DEBUG HELPERS
// ---------------------------------------------------------------------------
size_t dumpFirstNTags(uint16_t n, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    uint8_t s1, s2;
    if (!read8(REG_FIFO_STATUS1, s1) || !read8(REG_FIFO_STATUS2, s2)) return 0;
    uint16_t words = ((s2 & 0x03) << 8) | s1;  // unread FIFO words (TAG+6B)
    if (words >= n) break;
    delay(1);
  }

  size_t dumped = 0;
  for (; dumped < n; dumped++) {
    uint8_t raw[7];
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, raw, 7)) break;  // TAG @ 0x78
    uint8_t tag = raw[0];
    Serial.printf("TAG[%02u] = 0x%02X (id=%u)\n", (unsigned)dumped, tag, tag & 0x1F);
  }
  return dumped;
}

// --- Helpers to read UI vs. Embedded registers ---
bool readUI(uint8_t reg, uint8_t &val) {
  return read8(reg, val);  // normal UI bank read
}

bool readEmbedded(uint8_t reg, uint8_t &val) {
  if (!write8(REG_FUNC_CFG_ACCESS, 0x80)) return false;
  bool ok = read8(reg, val);
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok && true;
}

bool writeEmbedded(uint8_t reg, uint8_t val) {
  if (!write8(REG_FUNC_CFG_ACCESS, 0x80)) return false;
  bool ok = write8(reg, val);
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok && true;
}

bool shubWrite(uint8_t reg, uint8_t val) {
  if (!write8(REG_FUNC_CFG_ACCESS, SHUB_ACCESS)) return false;
  bool ok = write8(reg, val);
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok;
}

bool shubRead(uint8_t reg, uint8_t &val) {
  if (!write8(REG_FUNC_CFG_ACCESS, SHUB_ACCESS)) return false;
  bool ok = read8(reg, val);
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok;
}

void dumpSHUBConfigs() {
  uint8_t v;
  if (shubRead(REG_SLV0_CONFIG, v)) Serial.printf("SLV0_CONFIG (SHUB) = 0x%02X\n", v);
  if (shubRead(REG_SLV1_CONFIG, v)) Serial.printf("SLV1_CONFIG (SHUB) = 0x%02X\n", v);
  if (shubRead(REG_SLV2_CONFIG, v)) Serial.printf("SLV2_CONFIG (SHUB) = 0x%02X\n", v);
  if (shubRead(REG_SLV3_CONFIG, v)) Serial.printf("SLV3_CONFIG (SHUB) = 0x%02X\n", v);
}

// --- Fixed, comprehensive dump ---
void dumpIMURegisters_Fixed() {
  uint8_t v;

  auto rdUI = [&](uint8_t r, const char *n) {
    if (readUI(r, v)) Serial.printf("%-16s (0x%02X) = 0x%02X\n", n, r, v);
    else              Serial.printf("%-16s (0x%02X) = <read fail>\n", n, r);
  };
  auto rdEMB = [&](uint8_t r, const char *n) {
    if (readEmbedded(r, v)) Serial.printf("%-16s (0x%02X) = 0x%02X (EMB)\n", n, r, v);
    else                    Serial.printf("%-16s (0x%02X) = <read fail> (EMB)\n", n, r);
  };

  Serial.println("---- LSM6DSOX register dump (UI + Embedded) ----");
  uint8_t val = 0;
  read8(0x0F, val);
  Serial.printf("WHO_AM_I: %02X\n", val);

  rdUI(REG_CTRL1_XL,   "CTRL1_XL");
  rdUI(REG_CTRL2_G,    "CTRL2_G");
  rdUI(REG_CTRL3_C,    "CTRL3_C");
  rdUI(0x14,           "MASTER_CONFIG");
  rdUI(REG_FIFO_CTRL1, "FIFO_CTRL1");
  rdUI(REG_FIFO_CTRL2, "FIFO_CTRL2");
  rdUI(REG_FIFO_CTRL3, "FIFO_CTRL3");
  rdUI(REG_FIFO_CTRL4, "FIFO_CTRL4");
  rdUI(REG_FIFO_STATUS1, "FIFO_STATUS1");
  rdUI(REG_FIFO_STATUS2, "FIFO_STATUS2");

  rdEMB(REG_EMB_FUNC_EN_A, "EMB_FUNC_EN_A");
  rdEMB(REG_EMB_FUNC_EN_B, "EMB_FUNC_EN_B");
  dumpSHUBConfigs();

  Serial.println("------------------------------------------------");
}

bool setFuncCfg(bool enable) {
  return write8(REG_FUNC_CFG_ACCESS, enable ? 0x80 : 0x00);
}

void disableFifoCompression() {
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);

  setFuncCfg(true);
  write8(REG_EMB_FUNC_EN_B, 0x00);
  write8(REG_EMB_FUNC_EN_A, 0x00);
  setFuncCfg(false);
}

static int16_t signExtend5(int v) {
  v &= 0x1F;
  if (v & 0x10) {
    v |= ~0x1F;
  }
  return (int16_t)v;
}

// ---------------------------------------------------------------------------
// FIFO DECODE
// ---------------------------------------------------------------------------
size_t decodeFifoWord(fifoSample_t *sampleIn,
                      fifoSample_t *out,
                      size_t maxOut) {
  if (maxOut == 0) return 0;

  uint8_t tagField = (sampleIn->tag & TAG_SENSOR_MASK) >> TAG_SENSOR_SHIFT;

  SensorType     sensor = getSensorTypeFromTag(tagField);
  CompressionType comp  = getCompressionTypeFromTag(tagField);

  if (sensor == SENSOR_NONE) {
    return 0;
  }

  int16_t *last_x, *last_y, *last_z;
  bool    *have_last;
  uint8_t *data = sampleIn->payload;

  static_assert(sizeof(FifoDecompState::ax_last) == sizeof(int16_t), "field types");

  if (sensor == SENSOR_ACCEL) {
    last_x    = &decomp.ax_last;
    last_y    = &decomp.ay_last;
    last_z    = &decomp.az_last;
    have_last = &decomp.have_accel;
  } else {
    last_x    = &decomp.gx_last;
    last_y    = &decomp.gy_last;
    last_z    = &decomp.gz_last;
    have_last = &decomp.have_gyro;
  }

  // Uncompressed
  if (comp == COMP_NC || comp == COMP_NC_T1 || comp == COMP_NC_T2) {
    if (maxOut < 1) return 0;

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);

    *last_x = x;
    *last_y = y;
    *last_z = z;
    *have_last = true;

    out[0].tag = sampleIn->tag;
    out[0].x   = x;
    out[0].y   = y;
    out[0].z   = z;

    return 1;
  }

  if (!*have_last) {
    return 0;
  }

  // 2x compressed
  if (comp == COMP_2X) {
    if (maxOut < 2) return 0;

    int16_t diff[6];
    get_diff_2x(diff, data);

    int16_t x0 = *last_x + diff[0];
    int16_t y0 = *last_y + diff[1];
    int16_t z0 = *last_z + diff[2];

    int16_t x1 = x0 + diff[3];
    int16_t y1 = y0 + diff[4];
    int16_t z1 = z0 + diff[5];

    out[0].tag = sampleIn->tag;
    out[0].x   = x0;
    out[0].y   = y0;
    out[0].z   = z0;

    out[1].tag = sampleIn->tag;
    out[1].x   = x1;
    out[1].y   = y1;
    out[1].z   = z1;

    *last_x = x1;
    *last_y = y1;
    *last_z = z1;
    *have_last = true;

    return 2;
  }

  // 3x compressed
  if (comp == COMP_3X) {
    if (maxOut < 3) return 0;

    int16_t diff[9];
    get_diff_3x(diff, data);

    int16_t x0 = *last_x + diff[0];
    int16_t y0 = *last_y + diff[1];
    int16_t z0 = *last_z + diff[2];

    int16_t x1 = x0 + diff[3];
    int16_t y1 = y0 + diff[4];
    int16_t z1 = z0 + diff[5];

    int16_t x2 = x1 + diff[6];
    int16_t y2 = y1 + diff[7];
    int16_t z2 = z1 + diff[8];

    out[0].tag = sampleIn->tag;
    out[0].x   = x0;
    out[0].y   = y0;
    out[0].z   = z0;

    out[1].tag = sampleIn->tag;
    out[1].x   = x1;
    out[1].y   = y1;
    out[1].z   = z1;

    out[2].tag = sampleIn->tag;
    out[2].x   = x2;
    out[2].y   = y2;
    out[2].z   = z2;

    *last_x = x2;
    *last_y = y2;
    *last_z = z2;
    *have_last = true;

    return 3;
  }

  return 0;
}

// ---------------------------------------------------------------------------
// FIFO READ
// ---------------------------------------------------------------------------
uint16_t getFIFOSize() {
  uint8_t diff_lo, status2;
  if (!read8(REG_FIFO_STATUS1, diff_lo) || !read8(REG_FIFO_STATUS2, status2))
    return 0;

  uint16_t fifo_level = ((status2 & 0x03) << 8) | diff_lo;
  return fifo_level;
}

size_t readFIFO(fifoSample_t *buffer, size_t maxSamples) {
  uint8_t diff_lo, status2;
  if (!read8(REG_FIFO_STATUS1, diff_lo) || !read8(REG_FIFO_STATUS2, status2))
    return 0;

  uint16_t fifo_level = ((status2 & 0x03) << 8) | diff_lo;
  if (fifo_level == 0)
    return 0;

  size_t outCount = 0;

  for (uint16_t i = 0; i < fifo_level; i++) {
    if (outCount >= maxSamples) break;

    uint8_t raw[7];
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, raw, 7)) break;

    fifoSample_t tmp;
    tmp.tag = raw[0];
    memcpy(tmp.payload, &raw[1], 6);

    size_t nDecoded = decodeFifoWord(&tmp,
                                     &buffer[outCount],
                                     maxSamples - outCount);
    outCount += nDecoded;
  }

  return outCount;
}

size_t readFIFONoDecode(fifoSample_t *buffer, size_t maxSamples) {
  uint8_t diff_lo, status2;
  if (!read8(REG_FIFO_STATUS1, diff_lo) || !read8(REG_FIFO_STATUS2, status2))
    return 0;

  uint16_t fifo_level = ((status2 & 0x03) << 8) | diff_lo;
  if (fifo_level == 0)
    return 0;

  size_t outCount = 0;

  for (uint16_t i = 0; i < fifo_level; i++) {
    if (outCount >= maxSamples) break;

    uint8_t raw[7];
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, raw, 7)) break;

    fifoSample_t *outputSample_p = &buffer[outCount];
    outputSample_p->tag = raw[0];
    memcpy(outputSample_p->payload, &raw[1], 6);

    outCount += 1;
  }

  return outCount;
}

// ---------------------------------------------------------------------------
// CONFIG
// ---------------------------------------------------------------------------
bool enableAccelGyro(uint16_t frequencyHz) {
  uint8_t odrBits = 0;

  if      (frequencyHz >= 6667) odrBits = 0b1010;
  else if (frequencyHz >= 3333) odrBits = 0b1001;
  else if (frequencyHz >= 1667) odrBits = 0b1000;
  else if (frequencyHz >= 833)  odrBits = 0b0111;
  else if (frequencyHz >= 416)  odrBits = 0b0110;
  else if (frequencyHz >= 208)  odrBits = 0b0101;
  else if (frequencyHz >= 104)  odrBits = 0b0100;
  else if (frequencyHz >= 52)   odrBits = 0b0011;
  else if (frequencyHz >= 26)   odrBits = 0b0010;
  else if (frequencyHz >= 12)   odrBits = 0b0001;
  else return false;

  uint8_t accelReg = ((odrBits << 4) | (0b11 << 2));  // FS_XL=±16g
  uint8_t gyroReg  = ((odrBits << 4) | (0b01 << 2));  // FS_G=500 dps

  if (!write8(REG_CTRL1_XL, accelReg)) return false;
  if (!write8(REG_CTRL2_G,  gyroReg))  return false;

  Serial.printf("Accel & Gyro enabled @ %d Hz (ODR bits 0x%X)\n", frequencyHz, odrBits);
  return true;
}

void resetAndBypass() {
  write8(REG_CTRL3_C, 0x01);  // SW_RESET
  delay(100);
  write8(REG_CTRL3_C, (1 << 6) | (1 << 2));

  write8(REG_FIFO_CTRL4, 0x00);
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);
}

// ---------------------------------------------------------------------------
// CALIBRATION
// ---------------------------------------------------------------------------
void calibrateIMU(size_t samples) {
  Serial.println("Calibrating... keep device completely still!");
  delay(1000);

  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  size_t accelCount = 0;
  size_t gyroCount  = 0;
  size_t count      = 0;

  fifoSample_t buf[32];

  memset(&decomp, 0, sizeof(decomp));
  memset(&calib,  0, sizeof(calib));

  fifoSample_t flushBuf[32];
  while (readFIFO(flushBuf, 32) > 0) {
    delay(5);
  }

  while (count < samples) {
    size_t n = readFIFO(buf, 32);
    if (n == 0) {
      delay(5);
      continue;
    }

    for (size_t i = 0; i < n; i++) {
      if (isAccelData(buf[i].tag)) {
        ax_sum += buf[i].x;
        ay_sum += buf[i].y;
        az_sum += buf[i].z;
        accelCount++;
      }

      if (isGyroData(buf[i].tag)) {
        gx_sum += buf[i].x;
        gy_sum += buf[i].y;
        gz_sum += buf[i].z;
        gyroCount++;
      }

      count++;
      if (count >= samples) break;
    }
  }

  if (accelCount > 0) {
    float ax_mean = (float)ax_sum / accelCount;
    float ay_mean = (float)ay_sum / accelCount;
    float az_mean = (float)az_sum / accelCount;

    calib.ax_off = ax_mean;
    calib.ay_off = ay_mean;
    calib.az_off = az_mean;   // remove gravity; we'll handle gravity separately
  } else {
    calib.ax_off = calib.ay_off = calib.az_off = 0;
  }

  if (gyroCount > 0) {
    calib.gx_off = (float)gx_sum / gyroCount;
    calib.gy_off = (float)gy_sum / gyroCount;
    calib.gz_off = (float)gz_sum / gyroCount;
  } else {
    calib.gx_off = calib.gy_off = calib.gz_off = 0;
  }

  Serial.println("\nCalibration done.");
  Serial.printf("Accel offsets (LSB): %.2f %.2f %.2f\n",
                calib.ax_off, calib.ay_off, calib.az_off);
  Serial.printf("Gyro offsets  (LSB): %.2f %.2f %.2f\n",
                calib.gx_off, calib.gy_off, calib.gz_off);

  delay(200);
  isCalibrated = true;
}

// Direct accel read (bypass FIFO)
void readAccelDirect(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[6];
  if (!readBytes(REG_OUTX_L_A, buf, 6)) {
    ax = ay = az = 0;
    return;
  }
  ax = (int16_t)(buf[1] << 8 | buf[0]);
  ay = (int16_t)(buf[3] << 8 | buf[2]);
  az = (int16_t)(buf[5] << 8 | buf[4]);
}

// ---------------------------------------------------------------------------
// TAG DECODERS
// ---------------------------------------------------------------------------
static SensorType getSensorTypeFromTag(uint8_t tag) {
  switch (tag) {
    case TAG_XL:
    case TAG_XL_UNCOMPRESSED_T_1:
    case TAG_XL_UNCOMPRESSED_T_2:
    case TAG_XL_COMPRESSED_2X:
    case TAG_XL_COMPRESSED_3X:
      return SENSOR_ACCEL;

    case TAG_GY:
    case TAG_GY_UNCOMPRESSED_T_1:
    case TAG_GY_UNCOMPRESSED_T_2:
    case TAG_GY_COMPRESSED_2X:
    case TAG_GY_COMPRESSED_3X:
      return SENSOR_GYRO;

    default:
      return SENSOR_NONE;
  }
}

static CompressionType getCompressionTypeFromTag(uint8_t tag) {
  switch (tag) {
    case TAG_XL_UNCOMPRESSED_T_2:
    case TAG_GY_UNCOMPRESSED_T_2:
      return COMP_NC_T2;

    case TAG_XL_UNCOMPRESSED_T_1:
    case TAG_GY_UNCOMPRESSED_T_1:
      return COMP_NC_T1;

    case TAG_XL_COMPRESSED_2X:
    case TAG_GY_COMPRESSED_2X:
      return COMP_2X;

    case TAG_XL_COMPRESSED_3X:
    case TAG_GY_COMPRESSED_3X:
      return COMP_3X;

    default:
      return COMP_NC;
  }
}

static void get_diff_2x(int16_t diff[6], const uint8_t input[6]) {
  for (uint8_t i = 0; i < 6u; i++) {
    diff[i] = (input[i] < 128u) ? (int16_t)input[i]
                                : (int16_t)input[i] - 256;
  }
}

static void get_diff_3x(int16_t diff[9], const uint8_t input[6]) {
  uint16_t decode_tmp;

  for (uint8_t i = 0; i < 3u; i++) {
    decode_tmp = (uint16_t)input[2u * i] | ((uint16_t)input[2u * i + 1] << 8);

    for (uint8_t j = 0; j < 3u; j++) {
      uint16_t utmp = (decode_tmp & ((uint16_t)0x1Fu << (5u * j))) >> (5u * j);
      int16_t  tmp  = (int16_t)utmp;
      diff[j + 3u * i] = (tmp < 16) ? tmp : (tmp - 32);
    }
  }
}

// ---------------------------------------------------------------------------
// SAMPLE PROCESSING
// ---------------------------------------------------------------------------
void processSample(const fifoSample_t &sample, float *dataOut) {
  if (isAccelData(sample.tag)) {
    dataOut[0] = (sample.x - calib.ax_off) * ACC_SENS_16G;  // mg
    dataOut[1] = (sample.y - calib.ay_off) * ACC_SENS_16G;
    dataOut[2] = (sample.z - calib.az_off) * ACC_SENS_16G;
  }

  if (isGyroData(sample.tag)) {
    dataOut[0] = (sample.x - calib.gx_off) * GYRO_SENS_500DPS / 1000.0f;  // dps
    dataOut[1] = (sample.y - calib.gy_off) * GYRO_SENS_500DPS / 1000.0f;
    dataOut[2] = (sample.z - calib.gz_off) * GYRO_SENS_500DPS / 1000.0f;
  }
}
