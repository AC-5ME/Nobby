#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
/*
#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1  //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#else
*/
#define DEV_I2C Wire  //Or Wire
#define SerialPort Serial
//#endif
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;

float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;

float mag_offsetX = (64.00 + -1068.00) / 2;  //(maxX + minX) mag calibration
float mag_offsetY = (568.00 + -519.00) / 2;  //(maxY + minY)
float mag_offsetZ = (829.00 + -270.00) / 2;  //(maxZ + minZ)

float accel_offsetX = (1215.00 + -1935.00) / 2;  //(maxX + minX) accel calibration
float accel_offsetY = (1651.00 + -1585.00) / 2;  //(maxY + minY)
float accel_offsetZ = (1319.00 + -1929.00) / 2;  //(maxZ + minZ)

long lastDisplayTime;

void setup() {
  SerialPort.begin(115200);

  DEV_I2C.begin();

  Acc.begin();
  Acc.Enable();
  Acc.EnableTemperatureSensor();
  Mag.begin();
  Mag.Enable();
}

void loop() {
  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);

  float temperature;
  Acc.GetTemperature(&temperature);

  int32_t magnetometer[3];
  Mag.GetAxes(magnetometer);

   float mag_x = magnetometer[0];
   float mag_y = magnetometer[1];
   float mag_z = magnetometer[2];

  int accel_x = accelerometer[0];
  int accel_y = accelerometer[1];
  int accel_z = accelerometer[2];

  int cal_magX = mag_x - mag_offsetX;  //Apply mag calibration offsets
  int cal_magY = mag_y - mag_offsetY;
  int cal_magZ = mag_y - mag_offsetZ;

  /*
  float calibrated_accelX = accel.getX() - accel_offsetX;  //Apply accel calibration offsets
  float calibrated_accelY = accel.getY() - accel_offsetY;
  float calibrated_accelZ = accel.getZ() - accel_offsetZ;

  if (mag_x < MagMinX) MagMinX = mag_x;     //determine absolute max/min
  if (mag_x > MagMaxX) MagMaxX = mag_x;

  if (mag_y < MagMinY) MagMinY = mag_y;
  if (mag_y > MagMaxY) MagMaxY = mag_y;

  if (mag_z < MagMinZ) MagMinZ = mag_z;
  if (mag_z > MagMaxZ) MagMaxZ = mag_z;
*/
  if (accel_x < AccelMinX) AccelMinX = accel_x;
  if (accel_x > AccelMaxX) AccelMaxX = accel_x;

  if (accel_y < AccelMinY) AccelMinY = accel_y;
  if (accel_y > AccelMaxY) AccelMaxY = accel_y;

  if (accel_z < AccelMinZ) AccelMinZ = accel_z;
  if (accel_z > AccelMaxZ) AccelMaxZ = accel_z;

  if ((millis() - lastDisplayTime) > 100) {

    int heading = atan2(cal_magY, cal_magX) * (180 / PI);
    if (heading < 0) {
      heading += 360;  // Convert to 0-360 degrees
    }
    Serial.print(heading);
    Serial.println(" degrees");

    /*
    Serial.print("Accel Minimums: ");
    Serial.print(AccelMinX);
    Serial.print("  ");
    Serial.print(AccelMinY);
    Serial.print("  ");
    Serial.print(AccelMinZ);
    Serial.println();
    Serial.print("Accel Maximums: ");
    Serial.print(AccelMaxX);
    Serial.print("  ");
    Serial.print(AccelMaxY);
    Serial.print("  ");
    Serial.print(AccelMaxZ);
    Serial.println();

   Serial.print("Mag Minimums: ");
    Serial.print(MagMinX);
    Serial.print("  ");
    Serial.print(MagMinY);
    Serial.print("  ");
    Serial.print(MagMinZ);
    Serial.println();
    Serial.print("Mag Maximums: ");
    Serial.print(MagMaxX);
    Serial.print("  ");
    Serial.print(MagMaxY);
    Serial.print("  ");
    Serial.print(MagMaxZ);
    Serial.println();
    Serial.println();
*/
    lastDisplayTime = millis();
  }
}
