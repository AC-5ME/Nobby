NOBBY!

![IMG_0548](https://github.com/user-attachments/assets/1713149b-5591-4d24-b4ff-895fcc3c8aa5)
![IMG_0531](https://github.com/user-attachments/assets/516c7c49-e209-4100-9990-588169252220)

Nobby is a multi-function, self-launching glider display and control system. It automates essential glider operations—making flights easier, safer, and more funner!

Features

Automated Cool-Down Timing: Tracks engine cool-down intervals for self-launching gliders.
eCompass (Digital Magnetometer): Real-time heading via LSM303AGR sensor.
Tailbeacon Controller: (Planned) Automates tailbeacon operation for safety/compliance.
Accelerometer: Monitors acceleration and orientation (via LSM303AGR).
Audio Input: (Planned) Integration with MAX 4466 for future acoustic/event sensing.

Hardware:
Microcontroller: ESP32 (M5Dial platform)
Magnetometer/Accelerometer: LSM303AGR

Microphone/Audio Sensor: MAX 4466
Display: Integrated with M5Dial

Project Status:

Core features (cool-down timer, eCompass) are implemented.
Tailbeacon control in development.

Getting Started:

Build Requirements

PlatformIO or Arduino IDE with ESP32 support
Original libraries for M5Dial: 1.0, M5GFX 0.2.8, M5Unified 0.2.0, LSM303AGR

Wiring

LSM303AGR connected via I2C to ESP32
MAX 4466 connected to ESP32 analog input
Display is integrated on the M5Dial

Upload and Run:
Use PlatformIO or Arduino IDE to build and flash the firmware to your M5Dial device.
Usage

Power on the device; the display will show compass initially
Use the built-in interface to navigate additional features.
(Planned) Tailbeacon will be accessible as development progresses.
