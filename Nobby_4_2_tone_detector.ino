#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include <Goertzel.h>

#define DEV_I2C Wire
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

enum State {
  timer,
  compass,
  beacon,
  enl_display,
  accel,
};
State currentState = enl_display;  //set default screen to ENL

void ACCEL();
void BEACON();
void COMPASS();
void DIGITS();  //data field display
void ENL_DISPLAY();
void ENL_MEASURE();
void PAGE();  //page label
void TIMER();

#define SAMPLE_RATE 2000
#define BLOCK_SIZE 300
#define NUM_TONES 1
#define ENL_PIN 2

float TONE_FREQS[NUM_TONES] = { 200 };  //IGC ENL 100-200Hz
float threshold = 3000;                 // Adjust for detection level
//bool ToneDetected = false;              //default motor off
volatile int samples[BLOCK_SIZE];
volatile bool samplesReady = false;

#define DETECT_CONSECUTIVE_BLOCKS 10    //require tone present for N consecutive blocks
int detectedBlocks[NUM_TONES] = { 0 };  // Counter for each tone

// Function to dynamically calibrate the ADC midpoint before each block
int calibrateMidpoint(int pin, int numSamples = 100) {
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);  // Short pause between samples
  }
  return (int)(sum / numSamples);
}

// Sampling task runs on a separate core (ESP32 FreeRTOS)
void IRAM_ATTR sampleTask(void* arg) {
  while (true) {
    // 1. Resample midpoint before block
    int adcMidpoint = calibrateMidpoint(ENL_PIN, 100);

    //Serial.print("adcMidpoint: ");
    //Serial.println(adcMidpoint);

    unsigned long t0 = micros();
    unsigned long nextSampleTime = t0;
    // 2. Sample BLOCK_SIZE points
    for (int i = 0; i < BLOCK_SIZE; i++) {
      while ((long)(micros() - nextSampleTime) < 0) {
        // busy-wait for precise timing (within task, doesn't block loop())
      }
      samples[i] = analogRead(ENL_PIN) - adcMidpoint;
      nextSampleTime += 1000000UL / SAMPLE_RATE;
    }
    samplesReady = true;
    vTaskDelay(1);  // Yield to other tasks
  }
}

uint8_t sx, sy;                //display center point
uint8_t bkgrnd_color = BLACK;  //background color

unsigned int previousMillis = 0;  // Store the last time the ENL counter was updated
uint8_t seconds = 0;              //ENL timer
uint8_t minutes = 0;
long lastDisplayTime;        //ENL display delay
long lastMeasureTime;        //ENL measure delay
uint8_t ENL_threshold = 90;  //motor running noise level %
int ENL_value = 0;
bool isENL_high = false;               //motor running default = off
bool isTimer_running = false;          //timer running bit
unsigned long countdownTime = 300000;  //5 minutes in milliseconds (300000)
unsigned long timeRemaining = 0;
unsigned long startTime = millis();  //cool down timer
unsigned long last_runTime = 5;      //5 sec delay & last run time to restart timer
long oldPosition = 50;               //encoder position

float accel_Z = 1.0;      //G meter
const float alpha = 0.1;  //(0 < alpha ≤ 1) Smaller values = smoother data but slower response
float smoothed_acc_z = 0.0;
int accel_samples = 10;  //accel calibration samples
int32_t sumZ = 0;

float mag_offsetX = (1351 + -1297) / 2;  //(maxX + minX) x_nucleo mag calibration installed 17 May 25
float mag_offsetY = (1261 + -1611) / 2;  //(maxY + minY)
float mag_offsetZ = (1828 + -1866) / 2;  //(maxZ + minZ)

float accel_offsetX = (1989.00 + -1988.00) / 2;  //(maxX + minX) x_nucleo accel calibration
float accel_offsetY = (1989.00 + -1969.00) / 2;  //(maxY + minY)
float accel_offsetZ = (1989.00 + -1988.00) / 2;  //(maxZ + minZ)

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);  //(cfg, encoder, RFID)
  M5.Display.setRotation(0);       //0-3 are clockwise rotations, 4-7 are counterclockwise rotations (installed @ 90 is 1)
  M5.Touch.setFlickThresh(50);     //flickThresh (distance)

  Serial.begin(115200);  //************disable before install************

  DEV_I2C.begin();
  Acc.begin();
  Acc.Enable();
  int32_t accelerometer[3];
  Acc.SetFS(LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_LOW_POWER_MODE);  //+-8G setting
  Acc.DisableTemperatureSensor();
  Mag.begin();
  Mag.Enable();

  for (int i = 0; i < accel_samples; i++) {  //calculate average offset for Z-axis
    Acc.GetAxes(accelerometer);
    sumZ += accelerometer[2];
  }

  int32_t avgZ = sumZ / accel_samples;  //calibrate G meter on start-up
  if (avgZ > 0) {
    accel_offsetZ = avgZ - 1000;  // Upright orientation
  } else {
    accel_offsetZ = avgZ + 1000;  // Inverted orientation
  }

  // Start the sampling task on core 0
  xTaskCreatePinnedToCore(
    sampleTask,    // Task function
    "SampleTask",  // Name
    4096,          // Stack size
    NULL,          // Params
    1,             // Priority
    NULL,          // Task handle
    0              // Core 0
  );

  sx = M5Dial.Display.width() / 2;  //center of display
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);           //set max brightness
  M5Dial.Display.setTextDatum(middle_center);  //sx, sy

  PAGE();
  M5Dial.Display.drawString("ENL", sx, sy - 100);  //start-up page
}

void DIGITS() {
  M5Dial.Display.setTextSize(2.25);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);
}

void ACCEL() {
  DIGITS();

  char G_buf[10];  //buffer to store G digits

  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);

  float acc_z = accelerometer[2] - accel_offsetZ;
  smoothed_acc_z = alpha * acc_z + (1 - alpha) * smoothed_acc_z;
  float G = -(smoothed_acc_z / 1000.0);

  snprintf(G_buf, sizeof(G_buf), " %.1f ", G);
  M5Dial.Display.drawString(G_buf, sx, sy + 20);
}

void BEACON() {
  DIGITS();

  M5Dial.Display.drawString("1202", sx, sy + 20);
}

void COMPASS() {
  DIGITS();

  int32_t magnetometer[3];
  int32_t accelerometer[3];
  Mag.GetAxes(magnetometer);
  Acc.GetAxes(accelerometer);

  float mag_x = magnetometer[0] - mag_offsetX;
  float mag_y = magnetometer[1] - mag_offsetY;
  float mag_z = magnetometer[2] - mag_offsetZ;

  float acc_x = accelerometer[0] - accel_offsetX;
  float acc_y = accelerometer[1] - accel_offsetY;
  float acc_z = accelerometer[2] - accel_offsetZ;

  float pitch = atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
  float roll = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z));

  float comp_x = mag_x * cos(pitch) + mag_z * sin(pitch);  //tilt compensation
  float comp_y = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll) - mag_z * sin(roll) * cos(pitch);

  int heading = atan2(comp_y, comp_x) * (180 / PI);
  if (heading < 0) {
    heading += 360;
  }

  if ((millis() - lastDisplayTime) > 500) {  // print heading delay
    switch (heading) {
      case 0 ... 23:
        M5Dial.Display.drawString("  N  ", sx, sy + 30);
        break;
      case 24 ... 65:
        M5Dial.Display.drawString(" NE ", sx, sy + 30);
        break;
      case 66 ... 110:
        M5Dial.Display.drawString("  E  ", sx, sy + 30);
        break;
      case 111 ... 156:
        M5Dial.Display.drawString(" SE ", sx, sy + 30);
        break;
      case 157 ... 202:
        M5Dial.Display.drawString("  S  ", sx, sy + 30);
        break;
      case 204 ... 247:
        M5Dial.Display.drawString(" SW ", sx, sy + 30);
        break;
      case 248 ... 293:
        M5Dial.Display.drawString("  W  ", sx, sy + 30);
        break;
      case 294 ... 336:
        M5Dial.Display.drawString(" NW ", sx, sy + 30);
        break;
      case 337 ... 360:
        M5Dial.Display.drawString("  N  ", sx, sy + 30);
        break;
    }
    lastDisplayTime = millis();
  }
}

void ENL_MEASURE() {
  static float window[BLOCK_SIZE];
  static bool hannReady = false;  // Precompute Hann window once
 bool ToneDetected = false;

  if (!hannReady) {
    for (int i = 0; i < BLOCK_SIZE; i++) {
      window[i] = 0.5 * (1 - cos(2 * PI * i / (BLOCK_SIZE - 1)));
    }
    hannReady = true;
  }

  if (samplesReady) {
    int localSamples[BLOCK_SIZE];
    noInterrupts();
    for (int i = 0; i < BLOCK_SIZE; i++) {
      localSamples[i] = samples[i];
    }
    samplesReady = false;
    interrupts();

    // Apply Hann window and convert to int for goertzel.h
    int winSamples[BLOCK_SIZE];
    for (int i = 0; i < BLOCK_SIZE; i++) {
      winSamples[i] = (int)(localSamples[i] * window[i]);
    }

    for (int i = 0; i < NUM_TONES; i++) {
      Goertzel goertzel(TONE_FREQS[i], SAMPLE_RATE);
      float mag = goertzel.Mag(winSamples, BLOCK_SIZE);

      if (mag > threshold) {
        detectedBlocks[i]++;
        //        Serial.print(detectedBlocks[i]);
        if (detectedBlocks[i] >= DETECT_CONSECUTIVE_BLOCKS) {
          //          Serial.print(TONE_FREQS[i]);
          ToneDetected = true;

          int currentMillis = millis();  //Get the current time and increment motor timer

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
        detectedBlocks[i] = 0;  // reset if not detected
      }
    }
  }

  if (!ToneDetected & !isTimer_running) {  //test for 2nd, 3rd... new motor cool down time
    PAGE();
    M5Dial.Display.drawString("TIMER", sx, sy - 50);
    currentState = timer;                           //switch to timer screen
    isTimer_running = true;                         //set bit cool down timer
    startTime = millis();                           //restart timer
    last_runTime = ((minutes * 60) + seconds) + 5;  //check that it's a new motor run + 5 secs of debounce
  }
}

void ENL_DISPLAY() {
  char timer_buf[10];  //ENL time

  snprintf(timer_buf, sizeof(timer_buf), " %-2dM %-2dS ", minutes, seconds);  //print ENL timer
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextFont(4);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.drawString(timer_buf, sx, sy - 75);

  if (ToneDetected) {
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
    M5Dial.Display.drawString("       ", sx, sy + 80);
    M5Dial.Display.drawString("RUNNING", sx, sy + 80);
  } else {
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.setTextFont(4);
    M5Dial.Display.setTextColor(RED, bkgrnd_color);
    M5Dial.Display.drawString("  STOPPED  ", sx, sy + 80);
  }
}

void PAGE() {                               //default page settings
  M5Dial.Display.fillScreen(bkgrnd_color);  //reset screen
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextColor(WHITE, bkgrnd_color);
  M5Dial.Display.setTextFont(4);
}

void TIMER() {
  char cool_buf[10];  //buffer to store timer digits
  int mins = 0;
  int secs = 0;

  if (!isTimer_running) {
    M5Dial.Display.setTextSize(0.8);
    M5Dial.Display.setTextFont(8);
    M5Dial.Display.drawString("05:00", sx, sy);
  }

  if (M5Dial.BtnA.isPressed()) {  //start/stop timer manually
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
      isTimer_running = false;  // Stop the timer
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(BLUE, bkgrnd_color);
      M5Dial.Display.setTextFont(8);
      M5Dial.Display.drawString("00:00", sx, sy);
    }
  }
}

void loop() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  ENL_MEASURE();  //check for motor running/stopped & start cool down timer

  if (t.state == 9) {  //swipe?
    delay(50);         //debounce swipe
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

  switch (currentState) {  //Call the page based on the state
    case timer:
      TIMER();
      break;
    case compass:
      COMPASS();
      break;
    case beacon:
      BEACON();
      break;
    case accel:
      ACCEL();
      break;
    case enl_display:
      ENL_DISPLAY();
      break;
  }
}