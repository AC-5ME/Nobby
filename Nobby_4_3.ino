#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <Goertzel.h>

// ================== CONSTANTS ===================
constexpr uint16_t ENL_SAMPLE_RATE = 2000;
constexpr uint16_t BLOCK_SIZE = 300;
constexpr uint8_t NUM_TONES = 1;
constexpr uint8_t ENL_PIN = 2;
constexpr float TONE_FREQS[NUM_TONES] = { 200.0f };  // IGC ENL 100-200Hz
constexpr float DETECTION_THRESHOLD = 3000.0f;
constexpr uint8_t DETECT_CONSECUTIVE_BLOCKS = 10;

// ================== DEVICES ===================
#define DEV_I2C Wire
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

// ================== STATE ===================
enum State {
  timer,
  compass,
  beacon,
  enl_display,
  accel,
};
State currentState = enl_display;  // Default to ENL display

// ================== GLOBALS ===================
volatile int samples[BLOCK_SIZE];
volatile bool samplesReady = false;
volatile bool ToneDetected = false;
int detectedBlocks[NUM_TONES] = { 0 };

// Display variables
uint8_t sx, sy;
uint8_t bkgrnd_color = BLACK;

// Timer and ENL variables
unsigned int previousMillis = 0;
uint8_t seconds = 0;
uint8_t minutes = 0;
long lastDisplayTime = 0;
uint8_t ENL_threshold = 90;
int ENL_value = 0;
bool isTimer_running = false;
unsigned long countdownTime = 300000;
unsigned long timeRemaining = 0;
unsigned long startTime = 0;
unsigned long last_runTime = 5;
long oldPosition = 50;

// Accelerometer & Magnetometer
float accel_Z = 1.0;
const float accel_alpha = 0.1;
float smoothed_acc_z = 0.0;
int accel_samples = 10;
int32_t sumZ = 0;
float smoothed_heading = 0.0;
const float heading_alpha = 0.05;  // Lower = smoother, but slower
float heading_offset = 0.0;        // Offset for calibration

float mag_offsetX = (64.00 + -1068) / 2;  //(maxX + minX) mag calibration
float mag_offsetY = (568 + -519) / 2;
float mag_offsetZ = (829 + -270) / 2;
/*
****Installed****
float mag_offsetX = (1351 + -1297) / 2;
float mag_offsetY = (1261 + -1611) / 2;
float mag_offsetZ = (1828 + -1866) / 2;
*/
float accel_offsetX = (1989.00 + -1988.00) / 2;
float accel_offsetY = (1989.00 + -1969.00) / 2;
float accel_offsetZ = (1989.00 + -1988.00) / 2;

// ================== FUNCTION DECLARATIONS ===================
void ACCEL();
void BEACON();
void COMPASS();
void DIGITS();
void ENL_DISPLAY();
void ENL_MEASURE();
void PAGE();
void TIMER();

/**
 * Dynamically calibrate the ADC midpoint before each block
 */
int calibrateMidpoint(int pin, int numSamples = 100) {
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum / numSamples);
}

/**
 * Sampling task runs on a separate core (ESP32 FreeRTOS)
 */
void IRAM_ATTR sampleTask(void* arg) {
  while (true) {
    int adcMidpoint = calibrateMidpoint(ENL_PIN, 100);
    unsigned long t0 = micros();
    unsigned long nextSampleTime = t0;
    for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
      while ((long)(micros() - nextSampleTime) < 0) {}
      samples[i] = analogRead(ENL_PIN) - adcMidpoint;
      nextSampleTime += 1000000UL / ENL_SAMPLE_RATE;
    }
    samplesReady = true;
    vTaskDelay(1);
  }
}

/**
 * Setup: Initialize hardware and state
 */
void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(0);
  M5.Touch.setFlickThresh(50);

  Serial.begin(115200);

  DEV_I2C.begin();
  Acc.begin();
  Acc.Enable();
  int32_t accelerometer[3];
  Acc.SetFS(LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_LOW_POWER_MODE);
  Acc.DisableTemperatureSensor();
  Mag.begin();
  Mag.Enable();

  for (int i = 0; i < accel_samples; i++) {
    Acc.GetAxes(accelerometer);
    sumZ += accelerometer[2];
  }
  int32_t avgZ = sumZ / accel_samples;
  accel_offsetZ = (avgZ > 0) ? (avgZ - 1000) : (avgZ + 1000);

  xTaskCreatePinnedToCore(
    sampleTask,
    "SampleTask",
    4096,
    NULL,
    1,
    NULL,
    0);

  sx = M5Dial.Display.width() / 2;
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);
  M5Dial.Display.setTextDatum(middle_center);

  PAGE();
  M5Dial.Display.drawString("ENL", sx, sy - 100);
}

/**
 * Set up display for digits
 */
void DIGITS() {
  M5Dial.Display.setTextSize(2.25);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);
}

/**
 * Display acceleration (G meter)
 */
void ACCEL() {
  DIGITS();
  char G_buf[10];
  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);

  float acc_z = accelerometer[2] - accel_offsetZ;
  smoothed_acc_z = accel_alpha * acc_z + (1 - accel_alpha) * smoothed_acc_z;
  float G = -(smoothed_acc_z / 1000.0);

  snprintf(G_buf, sizeof(G_buf), " %.1f ", G);
  M5Dial.Display.drawString(G_buf, sx, sy + 20);
}

/**
 * Display beacon channel (fixed string)
 */
void BEACON() {
  DIGITS();
  M5Dial.Display.drawString("1202", sx, sy + 20);
}

/**
 * Display compass heading (with tilt compensation)
 */
const char* getCompassDirection(int heading) {
  if (heading >= 0 && heading < 23) return "  N  ";
  if (heading < 68) return " NE ";
  if (heading < 113) return "  E  ";
  if (heading < 158) return " SE ";
  if (heading < 203) return "  S  ";
  if (heading < 248) return " SW ";
  if (heading < 293) return "  W  ";
  if (heading < 338) return " NW ";
  return "  N  ";
}

void COMPASS() {
  DIGITS();

  int32_t mag_raw[3], acc_raw[3];
  Mag.GetAxes(mag_raw);
  Acc.GetAxes(acc_raw);

  // Remove offsets
  float ax = acc_raw[0] - accel_offsetX;
  float ay = acc_raw[1] - accel_offsetY;
  float az = acc_raw[2] - accel_offsetZ;
  float mx = mag_raw[0] - mag_offsetX;
  float my = mag_raw[1] - mag_offsetY;
  float mz = mag_raw[2] - mag_offsetZ;

  // Normalize accelerometer vector
  float acc_norm = sqrt(ax * ax + ay * ay + az * az);
  if (acc_norm < 800 || acc_norm > 1200) {
    return;
  }
  ax /= acc_norm;
  ay /= acc_norm;
  az /= acc_norm;

  // Normalize magnetometer vector
  float mag_norm = sqrt(mx * mx + my * my + mz * mz);
  if (mag_norm < 1.0) {
    return;
  }
  mx /= mag_norm;
  my /= mag_norm;
  mz /= mag_norm;

  // Compute East and North vectors
  float ex = my * az - mz * ay;
  float ey = mz * ax - mx * az;
  float ez = mx * ay - my * ax;
  float e_norm = sqrt(ex * ex + ey * ey + ez * ez);
  if (e_norm == 0) {
    return;
  }
  ex /= e_norm;
  ey /= e_norm;
  ez /= e_norm;

  float nx = ay * ez - az * ey;
  float ny = az * ex - ax * ez;
  float nz = ax * ey - ay * ex;
  // North vector is normalized because acc and E are.

  // Heading calculation (atan2(East, North))
  float raw_heading = atan2(-ex, nx) * (180.0 / PI);
  if (raw_heading < 0) raw_heading += 360;
  //Serial.println(raw_heading);

  // Calibrate with button
  if (M5Dial.BtnA.wasPressed()) {
    heading_offset = raw_heading;
  }

  float calibrated_heading = raw_heading - heading_offset;
  calibrated_heading = fmod(calibrated_heading + 360.0, 360.0);

  // Smoothing
  float delta = calibrated_heading - smoothed_heading;
  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;
  smoothed_heading += heading_alpha * delta;
  smoothed_heading = fmod(smoothed_heading + 360.0, 360.0);

  if ((millis() - lastDisplayTime) > 500) {
    M5Dial.Display.drawString(getCompassDirection((int)smoothed_heading), sx, sy + 30);
    lastDisplayTime = millis();
  }
}

/**
 * Analyze ENL samples for tone detection & timer logic
 */
void ENL_MEASURE() {
  static float window[BLOCK_SIZE];
  static bool hannReady = false;
  if (!hannReady) {
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      window[i] = 0.5 * (1 - cos(2 * PI * i / (BLOCK_SIZE - 1)));
    hannReady = true;
  }

  if (samplesReady) {
    int localSamples[BLOCK_SIZE];
    noInterrupts();
    memcpy(localSamples, (const void*)samples, sizeof(samples));
    samplesReady = false;
    interrupts();

    int winSamples[BLOCK_SIZE];
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      winSamples[i] = (int)(localSamples[i] * window[i]);

    bool detected = false;
    for (uint8_t i = 0; i < NUM_TONES; i++) {
      Goertzel goertzel(TONE_FREQS[i], ENL_SAMPLE_RATE);
      float mag = goertzel.Mag(winSamples, BLOCK_SIZE);

      if (mag > DETECTION_THRESHOLD) {
        detectedBlocks[i]++;
        if (detectedBlocks[i] >= DETECT_CONSECUTIVE_BLOCKS) {
          detected = true;
          int currentMillis = millis();
          if (currentMillis - previousMillis >= 1000) {
            previousMillis = currentMillis;
            seconds++;
            if (seconds >= 60) {
              seconds = 0;
              minutes++;
            }
          }
        }
      } else {
        detectedBlocks[i] = 0;
      }
    }
    ToneDetected = detected;
  }

  if (!ToneDetected && !isTimer_running) {
    PAGE();
    M5Dial.Display.drawString("TIMER", sx, sy - 50);
    currentState = timer;
    isTimer_running = true;
    startTime = millis();
    last_runTime = ((minutes * 60) + seconds) + 5;
  }
}

/**
 * Display ENL timer and motor status
 */
void ENL_DISPLAY() {
  char timer_buf[10];
  snprintf(timer_buf, sizeof(timer_buf), " %-2dM %-2dS ", minutes, seconds);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextFont(4);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.drawString(timer_buf, sx, sy - 75);

  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextFont(4);
  if (ToneDetected) {
    M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
    M5Dial.Display.drawString("RUNNING", sx, sy + 80);
  } else {
    M5Dial.Display.setTextColor(RED, bkgrnd_color);
    M5Dial.Display.drawString("STOPPED", sx, sy + 80);
  }
}

/**
 * Default page settings
 */
void PAGE() {
  M5Dial.Display.fillScreen(bkgrnd_color);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextColor(WHITE, bkgrnd_color);
  M5Dial.Display.setTextFont(4);
}

/**
 * Show and manage cooldown timer
 */
void TIMER() {
  char cool_buf[10];
  int mins = 0, secs = 0;

  if (!isTimer_running) {
    M5Dial.Display.setTextSize(0.8);
    M5Dial.Display.setTextFont(8);
    M5Dial.Display.drawString("05:00", sx, sy);
  }

  if (M5Dial.BtnA.isPressed()) {
    delay(200);
    if (isTimer_running) {
      isTimer_running = false;
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextFont(8);
      M5Dial.Display.drawString("05:00", sx, sy);
    } else {
      startTime = millis();
      isTimer_running = true;
    }
  }

  if (isTimer_running) {
    unsigned long elapsedTime = millis() - startTime;
    unsigned long remainingTime = countdownTime - elapsedTime;
    mins = remainingTime / 60000;
    secs = (remainingTime % 60000) / 1000;

    M5Dial.Display.setTextSize(0.8);
    M5Dial.Display.setTextColor(WHITE, bkgrnd_color);
    M5Dial.Display.setTextFont(8);
    snprintf(cool_buf, sizeof(cool_buf), "%02d:%02d", mins, secs);
    M5Dial.Display.drawString(cool_buf, sx, sy);

    timeRemaining = (mins * 60) + secs;

    if (timeRemaining <= 0) {
      isTimer_running = false;
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(BLUE, bkgrnd_color);
      M5Dial.Display.setTextFont(8);
      M5Dial.Display.drawString("00:00", sx, sy);
    }
  }
}

/**
 * Main loop: update state, process touch, call screen handlers
 */
void loop() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  ENL_MEASURE();

  if (t.state == 9) {  // swipe
    delay(50);
    switch (currentState) {
      case timer:
        currentState = compass;
        PAGE();
        M5Dial.Display.drawString("HEADING", sx, sy - 50);
        break;
      case compass:
        currentState = beacon;
        PAGE();
        M5Dial.Display.drawString("BEACON", sx, sy - 50);
        break;
      case beacon:
        currentState = accel;
        PAGE();
        M5Dial.Display.drawString("ACCEL", sx, sy - 50);
        break;
      case accel:
        currentState = enl_display;
        PAGE();
        M5Dial.Display.drawString("ENL", sx, sy - 100);
        break;
      case enl_display:
        currentState = timer;
        PAGE();
        M5Dial.Display.drawString("TIMER", sx, sy - 50);
        break;
    }
  }

  switch (currentState) {
    case timer: TIMER(); break;
    case compass: COMPASS(); break;
    case beacon: BEACON(); break;
    case accel: ACCEL(); break;
    case enl_display: ENL_DISPLAY(); break;
  }
}