// ESP32 + LSM6DSOX — FIFO level monitor (XL + G, no reads)
// I2C pins: SDA = GPIO 3, SCL = GPIO 4
// Uses Adafruit LSM6DSOX for basic bring-up; FIFO configured via raw regs.

#include <Wire.h>
// #include <Adafruit_LSM6DSOX.h>

#define SENSOR_FREQ 104u

typedef struct FifoSample {
  uint8_t tag;    // FIFO tag byte
  int16_t x;
  int16_t y;
  int16_t z;
}fifoSample_t;

// Adafruit_LSM6DSOX lsm6;
const uint16_t fifo_capacity = 128;

// I2C addresses depend on SDO wiring
#define LSM6DSOX_ADDR_6A 0x6A
#define LSM6DSOX_ADDR_6B 0x6B
uint8_t imuAddr = LSM6DSOX_ADDR_6A;

// Registers
#define REG_CTRL3_C       0x12
#define REG_CTRL1_XL      0x10
#define REG_CTRL2_G       0x11  
#define REG_FIFO_CTRL1    0x07
#define REG_FIFO_CTRL2    0x08
#define REG_FIFO_CTRL3    0x09
#define REG_FIFO_CTRL4    0x0A
#define REG_FIFO_STATUS1  0x3A
#define REG_FIFO_STATUS2  0x3B
#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79  // up to 0x7E
#define REG_FUNC_CFG_ACCESS 0x01 
#define REG_EMB_FUNC_EN_A   0x04
#define REG_EMB_FUNC_EN_B   0x05
#define SHUB_ACCESS   0x40  // FUNC_CFG_ACCESS.SHUB_REG_ACCESS
#define REG_SLV0_CONFIG    0x17  // SHUB page
#define REG_SLV1_CONFIG    0x19
#define REG_SLV2_CONFIG    0x1B
#define REG_SLV3_CONFIG    0x1D


// ---- helpers for raw I2C register access ----
static bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool read8(uint8_t reg, uint8_t &val) {
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

size_t dumpFirstNTags(uint16_t n, uint32_t timeout_ms = 100) {
  // Poll DIFF_FIFO_[9:0] until we have at least n words, or timeout
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    uint8_t s1, s2;
    if (!read8(REG_FIFO_STATUS1, s1) || !read8(REG_FIFO_STATUS2, s2)) return 0;
    uint16_t words = ((s2 & 0x03) << 8) | s1; // unread FIFO words (TAG+6B)
    if (words >= n) break;
    delay(1);
  }

  size_t dumped = 0;
  for (; dumped < n; dumped++) {
    uint8_t raw[7];
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, raw, 7)) break; // TAG @ 0x78
    uint8_t tag = raw[0];
    Serial.printf("TAG[%02u] = 0x%02X (id=%u)\n", (unsigned)dumped, tag, tag & 0x1F);
  }
  return dumped;
}

// --- Helpers to read UI vs. Embedded registers ---
bool readUI(uint8_t reg, uint8_t &val) {
  return read8(reg, val); // normal UI bank read
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

bool shubWrite(uint8_t reg, uint8_t val){
  if (!write8(REG_FUNC_CFG_ACCESS, SHUB_ACCESS)) return false;
  bool ok = write8(reg, val);
  write8(REG_FUNC_CFG_ACCESS, 0x00);
  return ok;
}
bool shubRead(uint8_t reg, uint8_t &val){
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

  auto rdUI = [&](uint8_t r, const char* n){
    if (readUI(r, v))  Serial.printf("%-16s (0x%02X) = 0x%02X\n", n, r, v);
    else               Serial.printf("%-16s (0x%02X) = <read fail>\n", n, r);
  };
  auto rdEMB = [&](uint8_t r, const char* n){
    if (readEmbedded(r, v)) Serial.printf("%-16s (0x%02X) = 0x%02X (EMB)\n", n, r, v);
    else                    Serial.printf("%-16s (0x%02X) = <read fail> (EMB)\n", n, r);
  };

  Serial.println("---- LSM6DSOX register dump (UI + Embedded) ----");
  uint8_t val = 0;
  read8(0x0f, val);
  Serial.printf("WHO_AM_I: %x\n", val);
  // UI bank (normal)
  rdUI(REG_CTRL1_XL,     "CTRL1_XL");
  rdUI(REG_CTRL2_G,      "CTRL2_G");
  rdUI(REG_CTRL3_C,      "CTRL3_C");
  rdUI(0x14,"MASTER_CONFIG");
  rdUI(REG_FIFO_CTRL1,   "FIFO_CTRL1");
  rdUI(REG_FIFO_CTRL2,   "FIFO_CTRL2");
  rdUI(REG_FIFO_CTRL3,   "FIFO_CTRL3");
  rdUI(REG_FIFO_CTRL4,   "FIFO_CTRL4");
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

size_t readFIFO(fifoSample_t *buffer, size_t maxSamples) {
  uint8_t diff_lo, status2;
  if (!read8(REG_FIFO_STATUS1, diff_lo) || !read8(REG_FIFO_STATUS2, status2))
    return 0;
  uint16_t fifo_level = ((status2 & 0x03) << 8) | diff_lo;
  if (fifo_level == 0)
   return 0;
  Serial.printf("Fifo level: %d\n", fifo_level);

  size_t toRead = min((size_t)fifo_level, maxSamples);

  for (size_t i = 0; i < toRead; i++) {
    uint8_t data[7];
    // if (!read8(REG_FIFO_DATA_OUT_TAG, tag)) break;
    // if (!readBytes(REG_FIFO_DATA_OUT_X_L, data, 6)) break;
    if (!readBytes(REG_FIFO_DATA_OUT_TAG, data, 7)) break;
    buffer[i].tag = data[0];
    buffer[i].x = (int16_t)(data[2] << 8 | data[1]);
    buffer[i].y = (int16_t)(data[4] << 8 | data[3]);
    buffer[i].z = (int16_t)(data[6] << 8 | data[5]);
  }
  return toRead;
}

bool enableAccelGyro(uint16_t frequencyHz) {
  uint8_t odrBits = 0;

  // Map the requested frequency to register bits (datasheet Table 55)
  if      (frequencyHz >= 6667) odrBits = 0b1010; // 6.66 kHz
  else if (frequencyHz >= 3333) odrBits = 0b1001; // 3.33 kHz
  else if (frequencyHz >= 1667) odrBits = 0b1000; // 1.66 kHz
  else if (frequencyHz >= 833)  odrBits = 0b0111; // 833 Hz
  else if (frequencyHz >= 416)  odrBits = 0b0110; // 416 Hz
  else if (frequencyHz >= 208)  odrBits = 0b0101; // 208 Hz
  else if (frequencyHz >= 104)  odrBits = 0b0100; // 104 Hz
  else if (frequencyHz >= 52)   odrBits = 0b0011; // 52 Hz
  else if (frequencyHz >= 26)   odrBits = 0b0010; // 26 Hz
  else if (frequencyHz >= 12)   odrBits = 0b0001; // 12.5 Hz
  else                          return false;

  uint8_t accelReg = ((odrBits << 4) | (0b01 << 2)); // ODR_XL[3:0], FS_XL=01
  uint8_t gyroReg  = ((odrBits << 4) | (0b01 << 2)); // ODR_G[3:0], FS_G=01

  if (!write8(REG_CTRL1_XL, accelReg)) return false;
  if (!write8(REG_CTRL2_G, gyroReg))   return false;

  Serial.printf("Accel & Gyro enabled @ %d Hz (ODR bits 0x%X)\n", frequencyHz, odrBits);
  return true;
}

void resetAndBypass() {
  // SW reset (CTRL3_C.SW_RESET=1) then re-enable BDU + IF_INC
  write8(REG_CTRL3_C, 0x01);  // SW_RESET
  delay(15);
  write8(REG_CTRL3_C, (1<<6)/*BDU*/ | (1<<2)/*IF_INC*/);

  // Put FIFO in BYPASS while configuring
  write8(REG_FIFO_CTRL4, 0x00);  // MODE=000 (bypass), DEC_TS=00, ODR_T=00
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);
}

bool isGyroData(uint8_t tag){
  tag = tag & 0x1F;
  bool isGyro =
      (tag == 0x01) ||  // Gyro NC
      (tag == 0x0A) ||  // Gyro NC_T_2
      (tag == 0x0B) ||  // Gyro NC_T_1
      (tag == 0x0C) ||  // Gyro 2xC
      (tag == 0x0D);    // Gyro 3xC

      return isGyro;
}

bool isAccelData(uint8_t tag){
  tag = tag & 0x1F;
  bool isAccel =
      (tag == 0x02) ||  // Accel NC
      (tag == 0x06) ||  // Accel NC_T_2
      (tag == 0x07) ||  // Accel NC_T_1
      (tag == 0x08) ||  // Accel 2xC
      (tag == 0x09);    // Accel 3xC

  return isAccel;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  if(!Wire.begin(3,4,400000))
  {
    Serial.println("Could not start I2C Wire...");
    while(1);
  }

  resetAndBypass();
  write8(REG_CTRL3_C, (1 << 6) | (1 << 2)); // Ensure auto-increment + block data update
  write8(0x14, 0); // disable sensor hub
  shubWrite(REG_SLV0_CONFIG, 0x00);
  shubWrite(REG_SLV1_CONFIG, 0x00);
  shubWrite(REG_SLV2_CONFIG, 0x00);
  shubWrite(REG_SLV3_CONFIG, 0x00);
  disableFifoCompression();

  if(!enableAccelGyro(SENSOR_FREQ))
  {
    Serial.println("Could not set sensor freq and scale");
    while(1);
  } 
  write8(REG_FIFO_CTRL3, (0b0100 << 4) | (0b0100)); // FIFO_CTRL3: BDR_XL in [7:4], BDR_G in [3:0]; 0b0100 = 104 Hz
  write8(REG_FIFO_CTRL4, 0x0); // Clear all old data
  write8(REG_FIFO_CTRL4, 0x06); // FIFO_CTRL4: 1 = FIFO (stop-when-full), 0x6 = continous & overwrite old data
  
  // Get first handful of samples to verify config
  fifoSample_t dataArr[16] = {0};
  delay(50);
  dumpFirstNTags(16);
  dumpIMURegisters_Fixed();
  delay(3000);

  Serial.println("FIFO configured (FIFO mode, XL+G @104Hz). Printing FIFO level...");
  delay(3000);
}

void loop() {
  static fifoSample_t fifoBuf[fifo_capacity];
  static uint16_t xlSamples = 0;
  static uint16_t gSamples = 0;

  size_t n = readFIFO(fifoBuf, fifo_capacity);
  if (n > 0) {
    Serial.print("Read ");
    Serial.print(n);
    Serial.println(" FIFO samples:");
    for (size_t i = 0; i < n; i++) {
      
      if(isGyroData(fifoBuf[i].tag))
        gSamples+=1;
      if(isAccelData(fifoBuf[i].tag))
        xlSamples+=1;

      // Serial.print("Tag: 0x"); Serial.print(fifoBuf[i].tag & 0x1f, HEX);
      // Serial.println();
      // Serial.print("  X: "); Serial.print(fifoBuf[i].x);
      // Serial.print("  Y: "); Serial.print(fifoBuf[i].y);
      // Serial.print("  Z: "); Serial.println(fifoBuf[i].z);
    }

    Serial.printf("Total gyro samples: %d | Accel: %d\n\n", gSamples, xlSamples);
  } else {
    Serial.println("No samples in FIFO.");
  }
  // dumpIMURegisters_Fixed();
  delay(500);
}
