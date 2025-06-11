#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <Goertzel.h>

// ================== CONSTANTS ===================
constexpr uint16_t ENL_SAMPLE_RATE = 4000;
constexpr uint16_t BLOCK_SIZE = 300;
constexpr uint8_t NUM_TONES = 3;
constexpr uint8_t ENL_PIN = 2;                                                      //blue wire
constexpr float TONE_FREQS[NUM_TONES] = { 100.0f, 150.0f, 200.0f };                 // IGC ENL 100-200Hz
constexpr float DETECTION_THRESHOLD[NUM_TONES] = { 50000.0f, 50000.0f, 50000.0f };  // 100,000+ motor running
constexpr uint8_t DETECT_CONSECUTIVE_BLOCKS = 10;                                   //trigger level for ENL

constexpr int START_PAGE_IDX = 1;  // Start at compass page
constexpr int NO_SWIPE = -1;
constexpr unsigned long COOLDOWN_TIME_MS = 300000;  // 5 minutes in ms
constexpr int TASK_STACK_SIZE = 4096;
constexpr int TASK_PRIORITY = 1;
constexpr int TASK_CORE = 0;
constexpr int CALIBRATE_SAMPLES = 100;
constexpr int ACCEL_TEXT_W = 320;
constexpr int ACCEL_TEXT_H = 170;
constexpr int ACCEL_TEXT_Y_OFFSET = 5;
constexpr float ACCEL_TEXT_SCALE = 1.4;
constexpr int COMPASS_RADIUS = 110;
constexpr int COMPASS_JET_SIZE = 45;
constexpr int SWIPE_THRESHOLD = 40;
constexpr int FLICK_THRESHOLD = 50;
constexpr float ACCEL_ALPHA = 0.05f;  //smaller is smoother (was 0.8)
constexpr float HEADING_ALPHA = 0.05f;
constexpr int ACCEL_SAMPLES = 10;
constexpr uint8_t ENL_THRESHOLD = 90;
constexpr int BEACON_CHANNEL = 1202;

// Colors
constexpr uint16_t COLOR_BKGRND = BLACK;
constexpr uint16_t COLOR_JET = WHITE;
constexpr uint16_t COLOR_CARDINAL = TFT_ORANGE;
constexpr uint16_t COLOR_G_TEXT = TFT_GREEN;

// ================== DEVICES AND GLOBALS ===================
#define DEV_I2C Wire
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

// pointers for Sprites
LGFX_Sprite* spriteAccel = nullptr;
LGFX_Sprite* spriteCompass = nullptr;

Goertzel* goertzelArray[NUM_TONES];

volatile int samples[BLOCK_SIZE];
volatile bool samplesReady = false;
volatile bool ToneDetected = false;
int detectedBlocks[NUM_TONES] = { 0 };
int freqBlockCounts[NUM_TONES] = { 0 };

uint8_t sx, sy;
uint8_t bkgrnd_color = COLOR_BKGRND;

// Timer and ENL variables
unsigned int previousMillis = 0;
uint8_t seconds = 0;
uint8_t minutes = 0;
long lastDisplayTime = 0;
bool isTimer_running = false;
unsigned long countdownTime = COOLDOWN_TIME_MS;
unsigned long timeRemaining = 0;
unsigned long startTime = 0;
float lastDetectedMag[NUM_TONES] = { 0 };
//averager
float magSum[NUM_TONES] = { 0 };
int magCount[NUM_TONES] = { 0 };
float magAvg[NUM_TONES] = { 0 };

// Accelerometer & Magnetometer
float smoothed_acc_z = 0.0f;
int32_t sumZ = 0;
float smoothed_heading = 0.0f;
float heading_offset = 0.0f;
float maxG = -100.0f;
float minG = 100.0f;

/* Installed
float mag_offsetX = (1351 + -1297) / 2;
float mag_offsetY = (1261 + -1611) / 2;
float mag_offsetZ = (1828 + -1866) / 2;
*/

constexpr float mag_minX = -1068.0f, mag_maxX = 64.0f;  //couch
constexpr float mag_minY = -519.0f, mag_maxY = 568.0f;
constexpr float mag_minZ = -270.0f, mag_maxZ = 829.0f;

// 1. Hard-iron offset:
constexpr float mag_offsetX = (mag_maxX + mag_minX) / 2.0f;
constexpr float mag_offsetY = (mag_maxY + mag_minY) / 2.0f;
constexpr float mag_offsetZ = (mag_maxZ + mag_minZ) / 2.0f;

// 2. Per-axis "radius":
constexpr float mag_radiusX = (mag_maxX - mag_minX) / 2.0f;
constexpr float mag_radiusY = (mag_maxY - mag_minY) / 2.0f;
constexpr float mag_radiusZ = (mag_maxZ - mag_minZ) / 2.0f;

// 3. Average radius:
constexpr float avg_radius = (mag_radiusX + mag_radiusY + mag_radiusZ) / 3.0f;

// 4. Scale factors for each axis:
constexpr float mag_scaleX = avg_radius / mag_radiusX;
constexpr float mag_scaleY = avg_radius / mag_radiusY;
constexpr float mag_scaleZ = avg_radius / mag_radiusZ;

float accel_offsetX = (1989.0f + -1988.0f) / 2.0f;
float accel_offsetY = (1989.0f + -1969.0f) / 2.0f;
float accel_offsetZ = (1989.0f + -1988.0f) / 2.0f;

// Touch/swipe state
enum State {
  timer,
  compass,
  beacon,
  enl_display,
  accel,
};
constexpr State pageOrder[] = { timer, compass, beacon, accel, enl_display };
constexpr int NUM_PAGES = sizeof(pageOrder) / sizeof(pageOrder[0]);
int currentPageIdx = START_PAGE_IDX;
static int swipeStartX = NO_SWIPE;
static int swipeActive = 0;

// Forward declarations
void ACCEL();
void BEACON();
void COMPASS();
void ENL_DISPLAY();
void ENL_MEASURE();
void Page();
void TIMER();
void HeadingIndicator(float heading, int centerX, int centerY, int radius);
int calibrateMidpoint(int pin, int numSamples = CALIBRATE_SAMPLES);
void IRAM_ATTR sampleTask(void* arg);

// ================== IMPLEMENTATION ===================

void HeadingIndicator(float heading, int centerX, int centerY, int radius) {
  if (!spriteCompass) return;
  spriteCompass->fillSprite(COLOR_BKGRND);
  int sx = (radius + 5);
  int sy = (radius + 5);

  for (int angle = 0; angle < 360; angle += 30) {
    float dispAngle = (angle - heading) * DEG_TO_RAD;
    int xStart = sx + (radius - 10) * sin(dispAngle);
    int yStart = sy - (radius - 10) * cos(dispAngle);
    int xEnd = sx + (radius)*sin(dispAngle);
    int yEnd = sy - (radius)*cos(dispAngle);
    spriteCompass->drawLine(xStart, yStart, xEnd, yEnd, TFT_WHITE);

    if (angle % 90 != 0) {
      char buf[3];
      snprintf(buf, sizeof(buf), "%d", (angle == 0) ? 36 : angle / 10);
      int tx = sx + (radius - 25) * sin(dispAngle);
      int ty = sy - (radius - 25) * cos(dispAngle);
      spriteCompass->setTextColor(TFT_WHITE, COLOR_BKGRND);
      spriteCompass->drawCenterString(buf, tx, ty - 8, 2);
    }
  }
  struct {
    const char* label;
    int angle;
    uint16_t color;
  } cardinals[] = {
    { "N", 0, COLOR_CARDINAL }, { "E", 90, COLOR_CARDINAL }, { "S", 180, COLOR_CARDINAL }, { "W", 270, COLOR_CARDINAL }
  };
  for (auto& c : cardinals) {
    float dispAngle = (c.angle - heading) * DEG_TO_RAD;
    int tx = sx + (radius - 30) * sin(dispAngle);
    int ty = sy - (radius - 30) * cos(dispAngle);
    spriteCompass->setTextSize(1.5);
    spriteCompass->setTextColor(c.color, COLOR_BKGRND);
    spriteCompass->drawCenterString(c.label, tx, ty - 12, 4);
  }
  spriteCompass->setTextColor(TFT_WHITE, COLOR_BKGRND);
  int size = COMPASS_JET_SIZE;
  int cx = sx, cy = sy + 15;
  constexpr float px[] = {
    0.0, -0.13, -0.80, -0.80, -0.33, -0.27, -0.48, -0.25, -0.25, -0.08, 0.0,
    0.08, 0.25, 0.25, 0.48, 0.27, 0.33, 0.80, 0.80, 0.13, 0.0
  };
  constexpr float py[] = {
    -1.55, -0.70, -0.02, 0.13, 0.25, 0.37, 0.66, 0.66, 0.86, 0.73, 0.86,
    0.73, 0.86, 0.66, 0.66, 0.37, 0.25, 0.13, -0.02, -0.70, -1.55
  };
  constexpr int n = sizeof(px) / sizeof(float);
  for (int j = -1; j <= 1; ++j) {
    for (int i = 0; i < n - 1; ++i) {
      int x0 = cx + int(px[i] * size) + j;
      int y0 = cy + int(py[i] * size);
      int x1 = cx + int(px[i + 1] * size) + j;
      int y1 = cy + int(py[i + 1] * size);
      spriteCompass->drawLine(x0, y0, x1, y1, COLOR_JET);
    }
  }
  spriteCompass->pushSprite(centerX - sx, centerY - sy);
}

int calibrateMidpoint(int pin, int numSamples) {
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum / numSamples);
}

void IRAM_ATTR sampleTask(void* arg) {
  while (true) {
    int adcMidpoint = calibrateMidpoint(ENL_PIN, CALIBRATE_SAMPLES);
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

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(0);  //0-3 are clockwise rotations, 4-7 are counterclockwise rotations (installed @ 90 is 1)
  M5.Touch.setFlickThresh(FLICK_THRESHOLD);

  DEV_I2C.begin();
  Acc.begin();
  Acc.Enable();
  Acc.SetFS(LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_LOW_POWER_MODE);
  Acc.DisableTemperatureSensor();
  Mag.begin();
  Mag.Enable();

  //calibrate accel on start-up
  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    int32_t accelerometer[3];
    Acc.GetAxes(accelerometer);
    sumZ += accelerometer[2];
  }
  int32_t avgZ = sumZ / ACCEL_SAMPLES;
  accel_offsetZ = (avgZ > 0) ? (avgZ - 1000) : (avgZ + 1000);

  // Only create sprites ONCE
  if (!spriteAccel) {
    spriteAccel = new LGFX_Sprite(&M5Dial.Display);
    if (!spriteAccel->createSprite(ACCEL_TEXT_W, ACCEL_TEXT_H)) {
    }
    spriteAccel->setTextDatum(middle_center);
  }
  if (!spriteCompass) {
    spriteCompass = new LGFX_Sprite(&M5Dial.Display);
    if (!spriteCompass->createSprite(COMPASS_RADIUS * 2 + 10, COMPASS_RADIUS * 2 + 10)) {
    }
    spriteCompass->setTextDatum(middle_center);
  }

  xTaskCreatePinnedToCore(
    sampleTask,
    "SampleTask",
    TASK_STACK_SIZE,
    NULL,
    TASK_PRIORITY,
    NULL,
    TASK_CORE);

  for (uint8_t i = 0; i < NUM_TONES; i++) {
    goertzelArray[i] = new Goertzel(TONE_FREQS[i], ENL_SAMPLE_RATE);
    if (!goertzelArray[i]) {
      Serial.printf("Goertzel allocation failed for tone %d\n", i);
      while (1) {}
    }
  }

  sx = M5Dial.Display.width() / 2;
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);
  M5Dial.Display.setTextDatum(middle_center);
}

void ACCEL() {
  char G_buf[10];
  char minmax_buf[32];
  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);

  float acc_z = accelerometer[2] - accel_offsetZ;
  smoothed_acc_z = ACCEL_ALPHA * acc_z + (1.0f - ACCEL_ALPHA) * smoothed_acc_z;
  float G = -(smoothed_acc_z / 1000.0f);

  // Update min and max
  if (G > maxG) maxG = G;
  if (G < minG) minG = G;

  // Reset min/max if button pressed
  if (M5Dial.BtnA.isPressed()) {
    delay(200);
    maxG = G;
    minG = G;
  }

  snprintf(G_buf, sizeof(G_buf), " %.1f ", G);

  if (spriteAccel) {
    spriteAccel->fillSprite(COLOR_BKGRND);
    spriteAccel->setFont(&fonts::Font8);
    spriteAccel->setTextColor(WHITE, COLOR_BKGRND);
    spriteAccel->setTextSize(ACCEL_TEXT_SCALE);

    int x = ACCEL_TEXT_W / 2;
    int y = ACCEL_TEXT_H / 2;
    spriteAccel->drawString(G_buf, x, y - 20);

    // Display max and min G values
    spriteAccel->setTextColor(GREEN, COLOR_BKGRND);
    spriteAccel->setFont(&fonts::Font4);
    spriteAccel->setTextSize(1);
    snprintf(minmax_buf, sizeof(minmax_buf), "max: %.1f  min: %.1f", maxG, minG);
    spriteAccel->drawString(minmax_buf, x, y + 50);  // Adjust y offset as needed

    spriteAccel->pushSprite(sx - ACCEL_TEXT_W / 2, sy + ACCEL_TEXT_Y_OFFSET - ACCEL_TEXT_H / 2);
  }
}

void BEACON() {
  char beacon_buf[8];

  M5Dial.Display.setTextSize(2.25);
  M5Dial.Display.setTextColor(GREEN, COLOR_BKGRND);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);
  snprintf(beacon_buf, sizeof(beacon_buf), "%d", BEACON_CHANNEL);
  M5Dial.Display.drawString(beacon_buf, sx, sy);
}

void COMPASS() {
  int32_t mag_raw[3], acc_raw[3];
  Mag.GetAxes(mag_raw);
  Acc.GetAxes(acc_raw);

  float ax = acc_raw[0] - accel_offsetX;
  float ay = acc_raw[1] - accel_offsetY;
  float az = acc_raw[2] - accel_offsetZ;

  float mx = (mag_raw[0] - mag_offsetX) * mag_scaleX;
  float my = (mag_raw[1] - mag_offsetY) * mag_scaleY;
  float mz = (mag_raw[2] - mag_offsetZ) * mag_scaleZ;
  /*
  float mx = mag_raw[0] - mag_offsetX;
  float my = mag_raw[1] - mag_offsetY;
  float mz = mag_raw[2] - mag_offsetZ;
*/
  float acc_norm = sqrt(ax * ax + ay * ay + az * az);
  if (acc_norm < 800 || acc_norm > 1200) return;
  ax /= acc_norm;
  ay /= acc_norm;
  az /= acc_norm;

  float mag_norm = sqrt(mx * mx + my * my + mz * mz);
  if (mag_norm < 1.0) return;
  mx /= mag_norm;
  my /= mag_norm;
  mz /= mag_norm;

  float ex = my * az - mz * ay;
  float ey = mz * ax - mx * az;
  float ez = mx * ay - my * ax;
  float e_norm = sqrt(ex * ex + ey * ey + ez * ez);
  if (e_norm == 0) return;
  ex /= e_norm;
  ey /= e_norm;
  ez /= e_norm;

  float nx = ay * ez - az * ey;
  float ny = az * ex - ax * ez;
  float nz = ax * ey - ay * ex;

  float raw_heading = atan2(-ex, nx) * (180.0 / PI);
  if (raw_heading < 0) raw_heading += 360;

  if (M5Dial.BtnA.wasPressed()) {
    heading_offset = raw_heading;
  }

  float calibrated_heading = raw_heading - heading_offset;
  calibrated_heading = fmod(calibrated_heading + 360.0f, 360.0f);

  float delta = calibrated_heading - smoothed_heading;
  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;
  smoothed_heading += HEADING_ALPHA * delta;
  smoothed_heading = fmod(smoothed_heading + 360.0f, 360.0f);

  if ((millis() - lastDisplayTime) > 500) {
    HeadingIndicator(smoothed_heading, sx, sy, COMPASS_RADIUS);
    lastDisplayTime = millis();
  }
}

void ENL_MEASURE() {
  static float window[BLOCK_SIZE];
  static int localSamples[BLOCK_SIZE];
  static int winSamples[BLOCK_SIZE];

  static bool hannReady = false;
  if (!hannReady) {
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      window[i] = 0.5f * (1.0f - cos(2 * PI * i / (BLOCK_SIZE - 1)));
    hannReady = true;
  }

  if (samplesReady) {
    memcpy((void*)localSamples, (const void*)samples, sizeof(samples));
    samplesReady = false;

    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      winSamples[i] = (int)(localSamples[i] * window[i]);

    bool detected_any = false;
    for (uint8_t i = 0; i < NUM_TONES; i++) {
      float mag = goertzelArray[i]->Mag(winSamples, BLOCK_SIZE);
      magSum[i] += mag;
      magCount[i]++;
      if (magCount[i] == 10) {
        magAvg[i] = magSum[i] / 10.0f;
        magSum[i] = 0;
        magCount[i] = 0;
      }
      lastDetectedMag[i] = mag;  // (optional: keep if you want instant value too)

      bool detected_now = (mag > DETECTION_THRESHOLD[i]);
      if (detected_now) {
        detectedBlocks[i]++;
        if (detectedBlocks[i] == DETECT_CONSECUTIVE_BLOCKS) {
          freqBlockCounts[i]++;
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
}

void ENL_DISPLAY() {
  char timer_buf[10];
  snprintf(timer_buf, sizeof(timer_buf), " %-2dM %-2dS ", minutes, seconds);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextFont(4);
  M5Dial.Display.setTextColor(GREEN, COLOR_BKGRND);
  M5Dial.Display.drawString(timer_buf, sx, sy - 75);

  if (ToneDetected) {
    M5Dial.Display.setTextColor(GREEN, COLOR_BKGRND);
    M5Dial.Display.drawString("RUNNING", sx, sy + 80);
  } else {
    M5Dial.Display.setTextColor(RED, COLOR_BKGRND);
    M5Dial.Display.drawString("STOPPED", sx, sy + 80);
  }

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
    M5Dial.Display.setTextColor(WHITE, COLOR_BKGRND);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.drawString(freq_buf, sx, sy);

    // Display the magnitude below the frequency
    char mag_buf[32];
    snprintf(mag_buf, sizeof(mag_buf), "Avg Mag: %.1f", magAvg[detectedIdx]);
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextColor(TFT_YELLOW, COLOR_BKGRND);
    M5Dial.Display.drawString(mag_buf, sx, sy + 50);

  } else {
    M5Dial.Display.setTextColor(DARKGREY, COLOR_BKGRND);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.drawString("      ---      ", sx, sy);
  }
}

void Page() {
  M5Dial.Display.fillScreen(COLOR_BKGRND);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextColor(WHITE, COLOR_BKGRND);
  M5Dial.Display.setTextFont(4);
}

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
    M5Dial.Display.setTextColor(WHITE, COLOR_BKGRND);
    M5Dial.Display.setTextFont(8);
    snprintf(cool_buf, sizeof(cool_buf), "%02d:%02d", mins, secs);
    M5Dial.Display.drawString(cool_buf, sx, sy);

    timeRemaining = (mins * 60) + secs;

    if (timeRemaining <= 0) {
      isTimer_running = false;
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(BLUE, COLOR_BKGRND);
      M5Dial.Display.setTextFont(8);
      M5Dial.Display.drawString("00:00", sx, sy);
    }
  }
}

void loop() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();
  ENL_MEASURE();

  if (t.state == 1 && !swipeActive) {
    swipeStartX = t.x;
    swipeActive = 1;
  }
  if ((t.state == 0 || t.state == 3) && swipeActive && swipeStartX != NO_SWIPE) {
    int dx = t.x - swipeStartX;
    if (abs(dx) > SWIPE_THRESHOLD) {
      if (dx < 0) {
        currentPageIdx = (currentPageIdx - 1 + NUM_PAGES) % NUM_PAGES;
      } else {
        currentPageIdx = (currentPageIdx + 1) % NUM_PAGES;
      }
      Page();
      switch (pageOrder[currentPageIdx]) {
        case timer: M5Dial.Display.drawString("TIMER", sx, sy - 50); break;
        case compass: break;
        case beacon: M5Dial.Display.drawString("BEACON", sx, sy - 50); break;
        case accel: M5Dial.Display.drawString("ACCEL", sx, sy - 90); break;
        case enl_display: M5Dial.Display.drawString("ENL", sx, sy - 100); break;
      }
    }
    swipeStartX = NO_SWIPE;
    swipeActive = 0;
  }

  switch (pageOrder[currentPageIdx]) {
    case timer: TIMER(); break;
    case compass: COMPASS(); break;
    case beacon: BEACON(); break;
    case accel: ACCEL(); break;
    case enl_display: ENL_DISPLAY(); break;
  }
}