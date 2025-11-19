// ESP32 + LSM6DSOX — FIFO level monitor (XL + G, no reads)
// I2C pins: SDA = GPIO 3, SCL = GPIO 4
// Uses Adafruit LSM6DSOX for basic bring-up; FIFO configured via raw regs.

#include "I2C_Abstract.hpp"
#include "IMU.hpp"

#define SENSOR_FREQ 104

const uint16_t fifo_capacity = 128;



void setup() {
  Serial.begin(115200);
  delay(2000);

  if (!Wire.begin(3, 4, 400000)) {
    Serial.println("Could not start I2C Wire...");
    while (1)
      ;
  }

  resetAndBypass();

  write8(REG_CTRL3_C, (1 << 6) | (1 << 2));  // Ensure auto-increment + block data update
  write8(0x19 /*CTRL10_C*/, 0x00);           // Disable timestamps
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

  dumpIMURegisters_Fixed();
  delay(3000);
  calibrateIMU(300);
}

void loop() {
  static fifoSample_t fifoBuf[fifo_capacity];
  static uint16_t xlSamples = 0;
  static uint16_t gSamples = 0;

  size_t n = readFIFO(fifoBuf, fifo_capacity);
  if (n > 0) {
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
  delay(500);
}
