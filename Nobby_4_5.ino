#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <Goertzel.h>

// ================== CONSTANTS ===================
constexpr uint16_t ENL_SAMPLE_RATE = 4000;
constexpr uint16_t BLOCK_SIZE = 300;
constexpr uint8_t NUM_TONES = 3;
constexpr uint8_t ENL_PIN = 2;
constexpr float TONE_FREQS[NUM_TONES] = { 100.0f, 150.0f, 200.0f };               // IGC ENL 100-200Hz
constexpr float DETECTION_THRESHOLD[NUM_TONES] = { 10000.0f, 4000.0f, 2000.0f };  // Adjust values per frequency
constexpr uint8_t DETECT_CONSECUTIVE_BLOCKS = 10;
static portMUX_TYPE samplesMux = portMUX_INITIALIZER_UNLOCKED;
Goertzel* goertzelArray[NUM_TONES];

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
constexpr State pageOrder[] = { timer, compass, beacon, accel, enl_display };
constexpr int NUM_PAGES = sizeof(pageOrder) / sizeof(pageOrder[0]);
int currentPageIdx = 1; // start at compass

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

float mag_offsetX = (64.00 + -1068) / 2;  //(maxX + minX) mag couch calibration
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

// For multi-frequency ENL display mode
int freqBlockCounts[NUM_TONES] = { 0 };  // How many times each freq passed DETECT_CONSECUTIVE_BLOCKS

// ================== FUNCTION DECLARATIONS ===================
void ACCEL();
void BEACON();
void COMPASS();
void DIGITS();
void ENL_DISPLAY();
void ENL_MEASURE();
void PAGE();
void TIMER();

void HeadingIndicator(float heading, int centerX, int centerY, int radius) {
  // tick marks and numbers
  for (int angle = 0; angle < 360; angle += 30) {
    float dispAngle = (angle - heading) * DEG_TO_RAD;
    int xStart = centerX + (radius - 15) * sin(dispAngle);
    int yStart = centerY - (radius - 15) * cos(dispAngle);
    int xEnd = centerX + (radius)*sin(dispAngle);
    int yEnd = centerY - (radius)*cos(dispAngle);
    M5Dial.Display.drawLine(xStart, yStart, xEnd, yEnd, TFT_WHITE);

    if (angle % 90 != 0) {
      char buf[3];
      snprintf(buf, sizeof(buf), "%d", (angle == 0) ? 36 : angle / 10);
      int tx = centerX + (radius - 25) * sin(dispAngle);
      int ty = centerY - (radius - 25) * cos(dispAngle);
      M5Dial.Display.setTextColor(TFT_WHITE, BLACK);
      M5Dial.Display.drawCentreString(buf, tx, ty - 8, 2);
    }
  }

  // Cardinal points
  struct {
    const char* label;
    int angle;
    uint16_t color;
  } cardinals[] = {
    { "N", 0, TFT_YELLOW }, { "E", 90, TFT_YELLOW }, { "S", 180, TFT_YELLOW }, { "W", 270, TFT_YELLOW }
  };
  for (auto& c : cardinals) {
    float dispAngle = (c.angle - heading) * DEG_TO_RAD;
    int tx = centerX + (radius - 30) * sin(dispAngle);
    int ty = centerY - (radius - 30) * cos(dispAngle);
    M5Dial.Display.setTextSize(1.5);  // Increase the number for bigger text
    M5Dial.Display.setTextColor(c.color, BLACK);
    M5Dial.Display.drawCentreString(c.label, tx, ty - 12, 4);
  }
  M5Dial.Display.setTextColor(TFT_WHITE, BLACK);

  // Fighter jet outline
  int size = 45;  // Adjust for your display
  int cx = centerX, cy = centerY + 15;

  // Jet outline points
  float px[] = {
    0.0, -0.13, -0.80, -0.80, -0.33, -0.27, -0.48, -0.25, -0.25, -0.08, 0.0,
    0.08, 0.25, 0.25, 0.48, 0.27, 0.33, 0.80, 0.80, 0.13, 0.0
  };
  float py[] = {
    -1.55, -0.70, -0.02, 0.13, 0.25, 0.37, 0.66, 0.66, 0.86, 0.73, 0.86,
    0.73, 0.86, 0.66, 0.66, 0.37, 0.25, 0.13, -0.02, -0.70, -1.55
  };
  int n = sizeof(px) / sizeof(float);

  // Draw the outline (thick/bold)
  for (int j = -1; j <= 1; ++j) {
    for (int i = 0; i < n - 1; ++i) {
      int x0 = cx + int(px[i] * size) + j;
      int y0 = cy + int(py[i] * size);
      int x1 = cx + int(px[i + 1] * size) + j;
      int y1 = cy + int(py[i + 1] * size);
      M5Dial.Display.drawLine(x0, y0, x1, y1, WHITE);
    }
  }
}

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

  for (uint8_t i = 0; i < NUM_TONES; i++) {
    goertzelArray[i] = new Goertzel(TONE_FREQS[i], ENL_SAMPLE_RATE);
  }

  sx = M5Dial.Display.width() / 2;
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);
  M5Dial.Display.setTextDatum(middle_center);
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

void COMPASS() {
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

  // Heading calculation (atan2(East, North))
  float raw_heading = atan2(-ex, nx) * (180.0 / PI);
  if (raw_heading < 0) raw_heading += 360;

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
    // Clear area (or whole screen)
    PAGE();
    // Draw aircraft-style heading indicator
    HeadingIndicator(smoothed_heading, sx, sy, 110);  // 110 for radius; adjust as needed
    lastDisplayTime = millis();
  }
}

/**
 * Analyze ENL samples for tone detection & timer logic
 * - Individual thresholds per frequency
 * - Flags/prints when each frequency is detected
 * - Tracks which freq most often meets DETECT_CONSECUTIVE_BLOCKS
 * - Serial prints magnitude and detectedBlocks for debug
 */
void ENL_MEASURE() {
  static float window[BLOCK_SIZE];
  static bool hannReady = false;
  static bool wasDetected[NUM_TONES] = { false };

  if (!hannReady) {
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      window[i] = 0.5 * (1 - cos(2 * PI * i / (BLOCK_SIZE - 1)));
    hannReady = true;
  }

  if (samplesReady) {
    int localSamples[BLOCK_SIZE];
    taskENTER_CRITICAL(&samplesMux);
    memcpy(localSamples, (const void*)samples, sizeof(samples));
    samplesReady = false;
    taskEXIT_CRITICAL(&samplesMux);

    int winSamples[BLOCK_SIZE];
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      winSamples[i] = (int)(localSamples[i] * window[i]);

    bool detected_any = false;
    for (uint8_t i = 0; i < NUM_TONES; i++) {
      float mag = goertzelArray[i]->Mag(winSamples, BLOCK_SIZE);

      bool detected_now = (mag > DETECTION_THRESHOLD[i]);

      /* Serial print debug including detectedBlocks count
      Serial.print("Freq: ");
      Serial.print(TONE_FREQS[i]);
      Serial.print(" Hz, Mag: ");
      Serial.print(mag);
      Serial.print(", DetectedBlocks: ");
      Serial.println(detectedBlocks[i]);
*/
      if (detected_now) {
        detectedBlocks[i]++;
        if (detectedBlocks[i] == DETECT_CONSECUTIVE_BLOCKS) {
          freqBlockCounts[i]++;
          // Serial.print("Freq ");
          //Serial.print(TONE_FREQS[i]);
          //Serial.print(" Hz reached 10 consecutive detections. Total: ");
          //Serial.println(freqBlockCounts[i]);
        }

        if (detectedBlocks[i] >= DETECT_CONSECUTIVE_BLOCKS) {
          detected_any = true;
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
    ToneDetected = detected_any;
  }

  /* Auto-start cooldown timer logic (optional)
  if (!ToneDetected && !isTimer_running && (((minutes * 60) + seconds) > last_runTime)) {
    PAGE();
    M5Dial.Display.drawString("TIMER", sx, sy - 50);
    currentState = timer;
    isTimer_running = true;
    startTime = millis();
    last_runTime = ((minutes * 60) + seconds) + 5;  // Adjust as appropriate
    }
*/
}

/**
 * Display only the freq most often above the detection threshold,
 * only after it has reached DETECT_CONSECUTIVE_BLOCKS at least once.
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

  // PATCH: Only show currently detected frequency with 10+ blocks
  int detectedIdx = -1;
  for (uint8_t i = 0; i < NUM_TONES; i++) {
    if (detectedBlocks[i] >= DETECT_CONSECUTIVE_BLOCKS) {
      detectedIdx = i;
      break;
    }
  }

  if (detectedIdx >= 0) {
    char freq_buf[24];
    snprintf(freq_buf, sizeof(freq_buf), "%.0f Hz", TONE_FREQS[detectedIdx]);
    M5Dial.Display.setTextColor(WHITE, bkgrnd_color);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.drawString(freq_buf, sx, sy);
  } else {
    M5Dial.Display.setTextColor(DARKGREY, bkgrnd_color);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.drawString("      ---      ", sx, sy);
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
  static unsigned long lastSwipeMillis = 0;
  const unsigned long swipeDebounceTime = 0;  // ms
  static int lastSwipeState = 0; // for rising edge detection

  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();
  //check for engine run
  ENL_MEASURE();
  
  // Only process on swipe state rising edge
  if (t.state == 9 && lastSwipeState != 9) {
    unsigned long now = millis();
    if (now - lastSwipeMillis > swipeDebounceTime) {
      lastSwipeMillis = now;
      if (t.x < sx) { // swipe left (go back)
        currentPageIdx = (currentPageIdx - 1 + NUM_PAGES) % NUM_PAGES;
      } else {        // swipe right (go forward)
        currentPageIdx = (currentPageIdx + 1) % NUM_PAGES;
      }
      PAGE();
      switch (pageOrder[currentPageIdx]) {
        case timer:        M5Dial.Display.drawString("TIMER", sx, sy - 50); break;
        case compass:      M5Dial.Display.drawString("HEADING", sx, sy - 50); break;
        case beacon:       M5Dial.Display.drawString("BEACON", sx, sy - 50); break;
        case accel:        M5Dial.Display.drawString("ACCEL", sx, sy - 50); break;
        case enl_display:  M5Dial.Display.drawString("ENL", sx, sy - 100); break;
      }
    }
  }
  lastSwipeState = t.state;

  // Call the appropriate handler
  switch (pageOrder[currentPageIdx]) {
    case timer: TIMER(); break;
    case compass: COMPASS(); break;
    case beacon: BEACON(); break;
    case accel: ACCEL(); break;
    case enl_display: ENL_DISPLAY(); break;
  }
}