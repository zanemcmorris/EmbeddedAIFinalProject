// ESP32 + LSM6DSOX — FIFO level monitor (XL + G, no reads)
// I2C pins: SDA = GPIO 3, SCL = GPIO 4
// Uses Adafruit LSM6DSOX for basic bring-up; FIFO configured via raw regs.

#include "I2C_Abstract.hpp"
#include "IMU.hpp"

#define SENSOR_FREQ 104u
// Accelerometer sensitivities (LSM6DSO datasheet)
const float ACC_SENS_16G = 0.488f;  // mg/LSB

// Gyro sensitivities
const float GYRO_SENS_500DPS = 17.50f;  // mdps/LSB


FifoDecompState decomp = { 0 };
Calibration calib;

// Adafruit_LSM6DSOX lsm6;
const uint16_t fifo_capacity = 128;

void processSample(const fifoSample_t &s) {
  static struct CombinedSample combined;
  uint8_t tag = s.tag & 0x1F;

  float ax, ay, az; // Acceleration for each dimension

  if (isAccelData(tag)) {
    // Conditionally apply offset
    // If the value is below the noise threshold, then it must be zero.
    // ax = (abs(s.x) < calib.ax_off)? 0 : (s.x - calib.ax_off) * ACC_SENS_16G;
    // ay = (abs(s.y) < calib.ay_off)? 0 : (s.y - calib.ay_off) * ACC_SENS_16G;
    // az = (abs(s.z) < calib.az_off)? 0 : (s.z - calib.az_off) * ACC_SENS_16G;

    // Original scaling
    ax = (s.x - calib.ax_off) * ACC_SENS_16G;  // mg
    ay = (s.y - calib.ay_off) * ACC_SENS_16G;
    az = (s.z - calib.az_off) * ACC_SENS_16G;

    combined.ax = ax;
    combined.ay = ay;
    combined.az = az;

    combined.hasAccel = true;
  }

  if (isGyroData(tag)) {
    float gx = (s.x - calib.gx_off) * GYRO_SENS_500DPS / 1000.0f;  // dps
    float gy = (s.y - calib.gy_off) * GYRO_SENS_500DPS / 1000.0f;
    float gz = (s.z - calib.gz_off) * GYRO_SENS_500DPS / 1000.0f;

    combined.gx = gx;
    combined.gy = gy;
    combined.gz = gz;

    combined.hasGyro = true;
  }

  // print combined only when both present
  if (combined.hasAccel && combined.hasGyro) {
    static uint32_t t0 = millis();
    uint32_t t = millis() - t0;

    Serial.printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",
                  combined.ax, combined.ay, combined.az,
                  combined.gx, combined.gy, combined.gz);

    memset(&combined, 0, sizeof(combined));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (!Wire.begin(3, 4, 400000)) {
    Serial.println("Could not start I2C Wire...");
    while (1)
      ;
  }

  resetAndBypass();
  setFuncCfg(true);
  uint8_t embB;
  readEmbedded(REG_EMB_FUNC_EN_B, embB);
  embB &= ~(1 << 7);  // disable FIFO compact mode
  writeEmbedded(REG_EMB_FUNC_EN_B, embB);
  setFuncCfg(false);
  write8(REG_CTRL3_C, (1 << 6) | (1 << 2));  // Ensure auto-increment + block data update
  write8(0x19 /*CTRL10_C*/, 0x00);           // Disable timestamps
  // write8(0x14, 0); // disable sensor hub
  shubWrite(REG_SLV0_CONFIG, 0x00);
  shubWrite(REG_SLV1_CONFIG, 0x00);
  shubWrite(REG_SLV2_CONFIG, 0x00);
  shubWrite(REG_SLV3_CONFIG, 0x00);
  disableFifoCompression();

  if (!enableAccelGyro(SENSOR_FREQ)) {
    Serial.println("Could not set sensor freq and scale");
    while (1)
      ;
  }
  // write8(REG_FIFO_CTRL2, 2);
  write8(REG_FIFO_CTRL3, (0b0100 << 4) | (0b0100));  // FIFO_CTRL3: BDR_XL in [7:4], BDR_G in [3:0]; 0b0100 = 104 Hz
  write8(REG_FIFO_CTRL4, 0x0);                       // Clear all old data
  write8(REG_FIFO_CTRL4, 0x06);                      // FIFO_CTRL4: 1 = FIFO (stop-when-full), 0x6 = continous & overwrite old data

  // Get first handful of samples to verify config
  // fifoSample_t dataArr[16] = { 0 };
  // delay(50);
  // dumpFirstNTags(16);
  dumpIMURegisters_Fixed();
  delay(3000);

  // Serial.println("FIFO configured (FIFO mode, XL+G @104Hz). Printing FIFO level...");
  // delay(3000);

  calibrateIMU(calib, decomp, 300);
}

void loop() {
  static fifoSample_t fifoBuf[fifo_capacity];
  static uint16_t xlSamples = 0;
  static uint16_t gSamples = 0;
  static bool headerPrinted = false;
  if (!headerPrinted) {
    Serial.println("t ax ay az gx gy gz");
    headerPrinted = true;
  }


  int16_t axr, ayr, azr;
  readAccelDirect(axr, ayr, azr);

  float ax_mg = axr * ACC_SENS_16G;
  float ay_mg = ayr * ACC_SENS_16G;
  float az_mg = azr * ACC_SENS_16G;

  Serial.printf("[DIRECT] raw: %6d %6d %6d  mg: %7.2f %7.2f %7.2f\n",
                axr, ayr, azr, ax_mg, ay_mg, az_mg);

  size_t n = readFIFO(fifoBuf, decomp, fifo_capacity);
  if (n > 0) {
    // Serial.print("Read ");
    // Serial.print(n);
    // Serial.println(" FIFO samples:");
    for (size_t i = 0; i < n; i++) {

      if (isGyroData(fifoBuf[i].tag))
        gSamples += 1;
      if (isAccelData(fifoBuf[i].tag))
        xlSamples += 1;

      // Serial.print("Tag: 0x"); Serial.print(fifoBuf[i].tag & 0x1f, HEX);
      // Serial.println();
      // Serial.print("  X: "); Serial.print(fifoBuf[i].x);
      // Serial.print("  Y: "); Serial.print(fifoBuf[i].y);
      // Serial.print("  Z: "); Serial.println(fifoBuf[i].z);

      processSample(fifoBuf[i]);
    }

    // Serial.printf("Total gyro samples: %d | Accel: %d\n\n", gSamples, xlSamples);
  } else {
    Serial.println("No samples in FIFO.");
  }
  // dumpIMURegisters_Fixed();
  delay(500);
}
