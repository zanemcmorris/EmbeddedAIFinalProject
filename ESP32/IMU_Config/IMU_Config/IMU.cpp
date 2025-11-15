#include "IMU.hpp"

bool isGyroData(uint8_t rawTag) {
  uint8_t tag = rawTag & 0x1F;
  switch (tag) {
    case 0x01:  // GYRO_NC
    case 0x0A:  // GYRO_NC_T_2
    case 0x0B:  // GYRO_NC_T_1
    case 0x0C:  // GYRO_2xC
    case 0x0D:  // GYRO_3xC
      return true;
    default:
      return false;
  }
}

bool isAccelData(uint8_t rawTag) {
  uint8_t tag = rawTag & 0x1F;
  switch (tag) {
    case 0x02:  // ACC_NC
    case 0x06:  // ACC_NC_T_2
    case 0x07:  // ACC_NC_T_1
    case 0x08:  // ACC_2xC
    case 0x09:  // ACC_3xC
      return true;
    default:
      return false;
  }
}


size_t dumpFirstNTags(uint16_t n, uint32_t timeout_ms) {
  // Poll DIFF_FIFO_[9:0] until we have at least n words, or timeout
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
  // Enter embedded-functions register space (FUNC_CFG_ACCESS.FUNC_CFG_EN=1)
  if (!write8(REG_FUNC_CFG_ACCESS, 0x80)) return false;
  bool ok = read8(reg, val);
  // Leave embedded space
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok && true;
}

// Optional: write helper for embedded bank if you need it elsewhere
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
    else Serial.printf("%-16s (0x%02X) = <read fail>\n", n, r);
  };
  auto rdEMB = [&](uint8_t r, const char *n) {
    if (readEmbedded(r, v)) Serial.printf("%-16s (0x%02X) = 0x%02X (EMB)\n", n, r, v);
    else Serial.printf("%-16s (0x%02X) = <read fail> (EMB)\n", n, r);
  };

  Serial.println("---- LSM6DSOX register dump (UI + Embedded) ----");
  uint8_t val = 0;
  read8(0x0f, val);
  Serial.printf("WHO_AM_I: %x\n", val);
  // UI bank (normal)
  rdUI(REG_CTRL1_XL, "CTRL1_XL");
  rdUI(REG_CTRL2_G, "CTRL2_G");
  rdUI(REG_CTRL3_C, "CTRL3_C");
  rdUI(0x14, "MASTER_CONFIG");
  rdUI(REG_FIFO_CTRL1, "FIFO_CTRL1");
  rdUI(REG_FIFO_CTRL2, "FIFO_CTRL2");
  rdUI(REG_FIFO_CTRL3, "FIFO_CTRL3");
  rdUI(REG_FIFO_CTRL4, "FIFO_CTRL4");
  rdUI(REG_FIFO_STATUS1, "FIFO_STATUS1");
  rdUI(REG_FIFO_STATUS2, "FIFO_STATUS2");

  // Embedded-functions bank (needs FUNC_CFG_ACCESS.FUNC_CFG_EN=1)
  rdEMB(REG_EMB_FUNC_EN_A, "EMB_FUNC_EN_A");
  rdEMB(REG_EMB_FUNC_EN_B, "EMB_FUNC_EN_B");
  dumpSHUBConfigs();

  Serial.println("------------------------------------------------");
}

bool setFuncCfg(bool enable) {
  // FUNC_CFG_ACCESS.FUNC_CFG_EN = 1 to enter, 0 to exit
  return write8(REG_FUNC_CFG_ACCESS, enable ? 0x80 : 0x00);
}

void disableFifoCompression() {
  uint8_t val = 0;
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);

  setFuncCfg(true);
  write8(REG_EMB_FUNC_EN_B, 0x00);
  write8(REG_EMB_FUNC_EN_A, 0x00);
  setFuncCfg(false);
}

static int16_t signExtend5(int v) {
  v &= 0x1F;       // keep 5 bits
  if (v & 0x10) {  // sign bit
    v |= ~0x1F;    // extend to 16 bits
  }
  return (int16_t)v;
}

size_t decodeFifoWord(uint8_t rawTag,
                      const uint8_t data[6],
                      fifoSample_t *out,
                      size_t maxOut,
                    FifoDecompState& decomp) {
  if (maxOut == 0) return 0;

  uint8_t tag = rawTag & 0x1F;

  bool isAccel = isAccelData(tag);
  bool isGyro = isGyroData(tag);

  if (!isAccel && !isGyro)
    return 0;

  // ------------ Select State ------------
  int16_t &S_last_x = isAccel ? decomp.ax_last : decomp.gx_last;
  int16_t &S_last_y = isAccel ? decomp.ay_last : decomp.gy_last;
  int16_t &S_last_z = isAccel ? decomp.az_last : decomp.gz_last;
  bool &have = isAccel ? decomp.have_accel : decomp.have_gyro;

  // ------------ 1) NC / NC_T1 / NC_T2 ------------
  if (tag == 0x02 || tag == 0x06 || tag == 0x07 || tag == 0x01 || tag == 0x0A || tag == 0x0B) {
    if (maxOut < 1) return 0;

    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];

    // NC gives us a new fully-known LAST sample
    S_last_x = x;
    S_last_y = y;
    S_last_z = z;
    have = true;

    out[0].tag = rawTag;
    out[0].x = x;
    out[0].y = y;
    out[0].z = z;
    return 1;
  }

  // ------------ If compressed but no previous sample → treat as NC -----------
  if (!have) {
    if (maxOut < 1) return 0;

    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];

    S_last_x = x;
    S_last_y = y;
    S_last_z = z;
    have = true;

    out[0].tag = rawTag;
    out[0].x = x;
    out[0].y = y;
    out[0].z = z;
    return 1;
  }

  // =====================================================================
  // 2) 2×C BLOCK
  // =====================================================================
  if (tag == 0x08 || tag == 0x0C)  // accel / gyro 2×C
  {
    if (maxOut < 2) return 0;

    // Read signed diffs
    int16_t d_i2_x = (int8_t)data[0];
    int16_t d_i2_y = (int8_t)data[1];
    int16_t d_i2_z = (int8_t)data[2];

    int16_t d_i1_x = (int8_t)data[3];
    int16_t d_i1_y = (int8_t)data[4];
    int16_t d_i1_z = (int8_t)data[5];

    // Base = S(i) = S_last_x/y/z — confirmed by ST
    int16_t Si_x = S_last_x;
    int16_t Si_y = S_last_y;
    int16_t Si_z = S_last_z;

    // Reconstruct:
    int16_t Sim1_x = Si_x + d_i1_x;
    int16_t Sim1_y = Si_y + d_i1_y;
    int16_t Sim1_z = Si_z + d_i1_z;

    int16_t Sim2_x = Sim1_x + d_i2_x;
    int16_t Sim2_y = Sim1_y + d_i2_y;
    int16_t Sim2_z = Sim1_z + d_i2_z;

    // Chronological order:
    out[0] = { rawTag, Sim2_x, Sim2_y, Sim2_z };  // (i-2)
    out[1] = { rawTag, Sim1_x, Sim1_y, Sim1_z };  // (i-1)

    // S(i) is still the newest (unchanged)
    return 2;
  }

  // =====================================================================
  // 3) 3×C BLOCK
  // =====================================================================
  if (tag == 0x09 || tag == 0x0D)  // accel / gyro 3×C
  {
    if (maxOut < 3) return 0;

    // Parse 5-bit packed diffs
    uint16_t wX = (data[1] << 8) | data[0];
    uint16_t wY = (data[3] << 8) | data[2];
    uint16_t wZ = (data[5] << 8) | data[4];

    int16_t d_i2_x = signExtend5(wX);
    int16_t d_i2_y = signExtend5(wX >> 5);
    int16_t d_i2_z = signExtend5(wX >> 10);

    int16_t d_i1_x = signExtend5(wY);
    int16_t d_i1_y = signExtend5(wY >> 5);
    int16_t d_i1_z = signExtend5(wY >> 10);

    int16_t d_i_x = signExtend5(wZ);
    int16_t d_i_y = signExtend5(wZ >> 5);
    int16_t d_i_z = signExtend5(wZ >> 10);

    // Base = S(i+1) = S_last — this is the critical fix!
    // ST AN5192: 3×C diffs are relative to the sample AFTER the block.
    int16_t Sip1_x = S_last_x;
    int16_t Sip1_y = S_last_y;
    int16_t Sip1_z = S_last_z;

    // Now reconstruct backwards:
    int16_t Si_x = Sip1_x - d_i_x;
    int16_t Si_y = Sip1_y - d_i_y;
    int16_t Si_z = Sip1_z - d_i_z;

    int16_t Sim1_x = Si_x - d_i1_x;
    int16_t Sim1_y = Si_y - d_i1_y;
    int16_t Sim1_z = Si_z - d_i1_z;

    int16_t Sim2_x = Sim1_x - d_i2_x;
    int16_t Sim2_y = Sim1_y - d_i2_y;
    int16_t Sim2_z = Sim1_z - d_i2_z;

    // Output oldest → newest
    out[0] = { rawTag, Sim2_x, Sim2_y, Sim2_z };
    out[1] = { rawTag, Sim1_x, Sim1_y, Sim1_z };
    out[2] = { rawTag, Si_x, Si_y, Si_z };

    // Update last sample to newest actual sample:
    S_last_x = Si_x;
    S_last_y = Si_y;
    S_last_z = Si_z;
    have = true;

    return 3;
  }

  return 0;
}


size_t readFIFO(fifoSample_t *buffer,  FifoDecompState &decomp, size_t maxSamples) {
  uint8_t diff_lo, status2;
  if (!read8(REG_FIFO_STATUS1, diff_lo) || !read8(REG_FIFO_STATUS2, status2))
    return 0;

  uint16_t fifo_level = ((status2 & 0x03) << 8) | diff_lo;
  if (fifo_level == 0)
    return 0;

  // fifo_level = number of FIFO "words" (tag+6B), not decoded samples
  Serial.printf("Fifo words: %d\n", fifo_level);

  size_t outCount = 0;

  for (uint16_t i = 0; i < fifo_level; i++) {
    if (outCount >= maxSamples) break;

    uint8_t raw[7];
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, raw, 7)) break;

    // if (isAccelData(raw[0])) {
    //   Serial.printf("TAG %02X RAW: %02X %02X %02X %02X %02X %02X\n",
    //                 raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
    // }

    size_t nDecoded = decodeFifoWord(raw[0], &raw[1],
                                     &buffer[outCount],
                                     maxSamples - outCount, decomp);
    outCount += nDecoded;
  }

  return outCount;
}

bool enableAccelGyro(uint16_t frequencyHz) {
  uint8_t odrBits = 0;

  // Map the requested frequency to register bits (datasheet Table 55)
  if (frequencyHz >= 6667) odrBits = 0b1010;       // 6.66 kHz
  else if (frequencyHz >= 3333) odrBits = 0b1001;  // 3.33 kHz
  else if (frequencyHz >= 1667) odrBits = 0b1000;  // 1.66 kHz
  else if (frequencyHz >= 833) odrBits = 0b0111;   // 833 Hz
  else if (frequencyHz >= 416) odrBits = 0b0110;   // 416 Hz
  else if (frequencyHz >= 208) odrBits = 0b0101;   // 208 Hz
  else if (frequencyHz >= 104) odrBits = 0b0100;   // 104 Hz
  else if (frequencyHz >= 52) odrBits = 0b0011;    // 52 Hz
  else if (frequencyHz >= 26) odrBits = 0b0010;    // 26 Hz
  else if (frequencyHz >= 12) odrBits = 0b0001;    // 12.5 Hz
  else return false;

  uint8_t accelReg = ((odrBits << 4) | (0b11 << 2));  // ODR_XL[3:0], FS_XL=01
  uint8_t gyroReg = ((odrBits << 4) | (0b01 << 2));   // ODR_G[3:0], FS_G=01

  if (!write8(REG_CTRL1_XL, accelReg)) return false;
  if (!write8(REG_CTRL2_G, gyroReg)) return false;

  Serial.printf("Accel & Gyro enabled @ %d Hz (ODR bits 0x%X)\n", frequencyHz, odrBits);
  return true;
}

void resetAndBypass() {
  // SW reset (CTRL3_C.SW_RESET=1) then re-enable BDU + IF_INC
  write8(REG_CTRL3_C, 0x01);  // SW_RESET
  delay(100);
  write8(REG_CTRL3_C, (1 << 6) /*BDU*/ | (1 << 2) /*IF_INC*/);

  // Put FIFO in BYPASS while configuring
  write8(REG_FIFO_CTRL4, 0x00);  // MODE=000 (bypass), DEC_TS=00, ODR_T=00
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);
}

void calibrateIMU(struct Calibration &calib, FifoDecompState &decomp, size_t samples) {
  Serial.println("Calibrating... keep device completely still!");
  delay(1000);

  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  size_t accelCount = 0;
  size_t gyroCount = 0;
  size_t count = 0;

  fifoSample_t buf[32];

  // reset decompressor state so old history doesn't pollute calibration
  memset(&decomp, 0, sizeof(decomp));
  fifoSample_t flushBuf[32];
  while (readFIFO(flushBuf, decomp, 32) > 0) {
    delay(5);
  }

  while (count < samples) {
    size_t n = readFIFO(buf, decomp, 32);
    if (n == 0) {
      delay(5);
      continue;
    }

    for (size_t i = 0; i < n; i++) {
      uint8_t tag = buf[i].tag & 0x1F;

      if (isAccelData(tag)) {
        ax_sum += buf[i].x;
        ay_sum += buf[i].y;
        az_sum += buf[i].z;
        accelCount++;
      }

      if (isGyroData(tag)) {
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
    // subtract 1 g worth of LSBs on Z so we keep +1g in az after calibration
    // calib.az_off = az_mean - (1000.0f / ACC_SENS_16G); // Calib step to KEEP gravity in acceleration
    calib.az_off = az_mean;  // REMOVE gravity in acceleration
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

  delay(2000);
}