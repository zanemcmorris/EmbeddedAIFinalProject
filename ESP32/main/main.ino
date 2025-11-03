#include <Adafruit_LSM6DSOX.h>

#define USE_PLOTTER 1

Adafruit_LSM6DSOX lsm6dsox;
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 25; // Sample every 25ms
sensors_event_t accel, gyro, temp;

void setup() {
  Serial.begin(115200);
  /* while (!Serial) {
    delay(10);
  }*/

  // Initialize IMU-related pins
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  // Initialize I2C
  Wire.begin(3, 4);
  delay(100);

  Serial.println("Attempting to initialize LSM6DSOX...");
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    Serial.println("Check your wiring!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");
  delay(1000);

  // Configure IMU settings
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // Print labels for Serial Plotter
  Serial.println("AccelX:AccelY:AccelZ:GyroX:GyroY:GyroZ:Temp");
}

void sampleIMUData() {
  
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  // Format data for serial plotter (label:value:label:value format)
  Serial.print("AccelX:");
  Serial.print(accel.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accel.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accel.acceleration.z);
  Serial.print(",");
  Serial.print("GyroX:");
  Serial.print(gyro.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyro.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(gyro.gyro.z);
  Serial.print(",");
  Serial.println(temp.temperature);
}

void loop() {
  unsigned long currentTime = millis();

  // Sample data at specified interval
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    sampleIMUData();
    lastSampleTime = currentTime;
  }
}