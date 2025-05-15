
#include <LSM303AGR_ACC_Sensor.h>

#define DEV_I2C Wire    //Or Wire
#define SerialPort Serial

// Components.
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);

float accel_offsetZ = (1989.00 + -1988.00) / 2;  //(maxZ + minZ)
float smoothed_acc_z = 0.0;
const float alpha = 0.1;  // Smoothing factor (0 < alpha ≤ 1)

void setup() {
Acc.SetFS(LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_NORMAL_MODE);
//Acc.GetFS();    //GetSensitivity
  SerialPort.begin(115200);
  
  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initlialize components.
  Acc.begin();
  Acc.Enable();

   int32_t accelerometer[3];

  // Take multiple readings to average the offset
  int samples = 10;
  int32_t sumZ = 0;

  for (int i = 0; i < samples; i++) {
    Acc.GetAxes(accelerometer);
    sumZ += accelerometer[2];
    delay(50);  // Small delay between readings
  }

  // Calculate average offset for Z-axis
  // Use the absolute value of 1G (1000 mg) to account for orientation
  int32_t avgZ = sumZ / samples;
  if (avgZ > 0) {
    accel_offsetZ = avgZ - 1000;  // Upright orientation
  } else {
    accel_offsetZ = avgZ + 1000;  // Inverted orientation
  }

}

void loop() {
    char G_buf[10];        //buffer to store G digits

  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);
  float acc_z = accelerometer[2] - accel_offsetZ;
  smoothed_acc_z = alpha * acc_z + (1 - alpha) * smoothed_acc_z;

  float G = -(smoothed_acc_z / 1000.0);


  snprintf(G_buf, sizeof(G_buf), " %.1fG", G);
 Serial.println(G);
}