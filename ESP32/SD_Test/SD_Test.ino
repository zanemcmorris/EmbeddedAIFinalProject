#include <Adafruit_LSM6DSOX.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

Adafruit_LSM6DSOX lsm6ds3trc;
File dataFile, settingsFile;
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 25; // Log every 25ms
const char* fileName = "/DATA.TXT";
volatile bool buttonFallingEdgeDetected = false;
bool loggingActive = false;
unsigned long loggingDisableTime = 0;
unsigned long loggingActivePeriod_ms = 1000;
unsigned int logNumber = 0;

void buttonISR()
{
  buttonFallingEdgeDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize I2C
  Wire.begin();
  delay(100);

  // Initialize SD card
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  // Initialize LSM6DS3TR-C
  Serial.println("Initializing LSM6DS3TR-C...");
  if (!lsm6ds3trc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DS3TR-C Found!");

  // Configure IMU settings
  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ);
  
  Serial.println("Logging started!");
  Serial.println("Format: timestamp,temperature,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");

  pinMode(0, INPUT_PULLUP);
  attachInterrupt(0, buttonISR, FALLING);

}

void logIMUData(bool isFirstEntry) {
  static int logNumber = 0;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  
  // Format data string
  char dataString[100];
  snprintf(dataString, sizeof(dataString), 
           "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           millis(),
           temp.temperature,
           accel.acceleration.x,
           accel.acceleration.y,
           accel.acceleration.z,
           gyro.gyro.x,
           gyro.gyro.y,
           gyro.gyro.z);

  // Create/Open data file and write header
  dataFile = SD.open(fileName, "a");
  if (!dataFile) {
    Serial.println("Failed to open data file");
    return;
  }
  
  // Write CSV header
  if(isFirstEntry){
    dataFile.println("timestamp,temperature,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
    dataFile.flush();
  }
  
  // Write to SD card
  dataFile.println(dataString);
  dataFile.flush();  // Ensure data is written to card
  
  // Also print to Serial for monitoring
  Serial.println(dataString);
}

void finishLoggingIMUData()
{
  // Create/Open data file and write header
  dataFile = SD.open(fileName, "a");
  if (!dataFile) {
    Serial.println("Failed to open data file");
    return;
  }

  dataFile.println("XXX");

  dataFile.close();
}

void loop() {
  static int numSamples = 0;
  unsigned long currentTime = millis();
  if(buttonFallingEdgeDetected){
    if(digitalRead(0) == true){
      // Button was released rapidly, reject
    } else {
      // Button is still pressed, likely a good input
      loggingActive = true; 
      loggingDisableTime = millis() + loggingActivePeriod_ms;
      Serial.println("Starting to log...");
    }

    buttonFallingEdgeDetected = false;
  }
  
  if(loggingActive && currentTime >= loggingDisableTime){
    loggingActive = false;
    finishLoggingIMUData();
    Serial.printf("Finished logging. Got %d samples\n", numSamples);
    numSamples = 0;
  }

  // Log data at specified interval
  if (loggingActive && currentTime - lastLogTime >= LOG_INTERVAL) {
    logIMUData(numSamples == 0);
    numSamples += 1;
    lastLogTime = currentTime;
  }
}