#  Swing Sense: AI Powered Smart Watch

The goal of the Swing Sense is to leverage an existing hardware solution (TinyCore) to create a wearable device that can recognize and relay information about the user's golf swings.

# How to run the project
First, you ought to get your hand on a TinyCore device with an Espressif ESP32-S3 and the STM LSM6DSO IMU. Next, follow the TinyCore documentation for installing the board in the Arduino IDE, the preferred developemt environment for the project. Then, navigate to the magic_wand_integration sketch and upload it to the board. The program will start automatically and start detecting movement strokes and publishing data to the companion webpage. You can view this webpage on a host computer by navigating to /ESP32/magic_wand_integration/website/index.html. From there you should see only one bluetooth device to connect to. Then you should see your movements reflected on the live monitor.

# File Structure

## ESP32
The ESP32 directory contains the various arduino IDE sketches that were used during development.

### IMU_Config
This sketch was the primary development region for the IMU driver. It has no calls to any sort of AI models and just contains the functions needed to configure and read data from the IMU.

### IMU_Test
This is a library example for the IMU that simply reads data streaming from the device. It can be used to verify that the IMU is set up properly on the board and that the I2C bus and other peripherals are functioning. It ought to just be a known-good example

### magic_wand
This is the base code from Pete Warden's magic wand example. It contains all of his documentation, examples, and tooling to get his magic wand working.

### magic_wand_integration
This is the final piece of software that mates the AI-driven magic wand with the new IMU driver that we developed. When we mention the 'final application', this is it. It contains many files that abstract away some of the IMU functionalities and customizes Pete Warden's example to run on the TinyCore ESP32-S3.

## TinyDisplay
This directory contains the early PCB designs of the TinyDisplay peripheral we sought to develop. We quickly moved the screen to being out of scope for the project so that we could work on other, more crucial, aspects of the project.

## tinyCore-Main
This is a clone of the TinyCore repo, which serves as a central location for all the TinyCore documentation, code examples, hardare layout, and more. It is here for quick reference.
