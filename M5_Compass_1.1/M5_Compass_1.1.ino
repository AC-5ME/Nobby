#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

float offsetX = (5.70 + -103.80) / 2;     //(maxX + minX) mag calibration
float offsetY = (55.05 + -45.90) / 2;     //(maxY + minY)
float offsetZ = (81.75 + -25.05) / 2;     //(maxZ + minZ)

long lastDisplayTime;

void setup(void) {
  Serial.begin(115200);
  while (!Serial);

  mag.begin();

  lastDisplayTime = millis();
}

void loop(void) {
  sensors_event_t magEvent;

  mag.getEvent(&magEvent);

  float calibratedX = magEvent.magnetic.x - offsetX;  //Apply calibration offsets
  float calibratedY = magEvent.magnetic.y - offsetY;
  float calibratedZ = magEvent.magnetic.z - offsetZ;

  if ((millis() - lastDisplayTime) > 500)  //interrupt
  {
    float heading = atan2(calibratedY, calibratedX) * (180 / PI);
    if (heading < 0) {
      heading += 360;  // Convert to 0-360 degrees
    }
    Serial.print(heading);

    Serial.println(" degrees");

    lastDisplayTime = millis();
  }
}
