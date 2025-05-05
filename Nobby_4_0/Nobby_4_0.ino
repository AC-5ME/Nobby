
#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>

#define DEV_I2C Wire
#define SerialPort Serial

LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

enum State {
  timer,
  compass,
  beacon,
  enl_display,
};
State currentState = enl_display;  //set default screen to ENL

void PAGE();  //page display settings
void TIMER();
void COMPASS();
void BEACON();
void ENL_DISPLAY();
void ENL_MEASURE();

uint8_t sx, sy;                //display center point
uint8_t bkgrnd_color = BLACK;  //background color

unsigned int previousMillis = 0;  // Store the last time the ENL counter was updated
uint8_t seconds = 0;              //ENL timer
uint8_t minutes = 0;
long lastDisplayTime;        //ENL display delay
long lastMeasureTime;        //ENL measure delay
uint8_t ENL_threshold = 90;  //motor running noise level %
uint8_t ENL_pin = 2;         //ENL pin, blue wire
int ENL_value = 0;
bool isENL_high = false;               //motor running default = off
bool isTimer_running = false;          //timer running bit
unsigned long countdownTime = 300000;  //5 minutes in milliseconds
unsigned long startTime = millis();    //cool down timer
unsigned long last_runTime = 5;        //5 sec delay & last run time to restart timer
long oldPosition = 50;                 //encoder position

float mag_offsetX = (291 + -861) / 2;  //(maxX + minX) x_nucleo mag calibration
float mag_offsetY = (351 + -738) / 2;  //(maxY + minY)
float mag_offsetZ = (853 + -277) / 2;  //(maxZ + minZ)

//float accel_offsetX = (1989.00 + -1988.00) / 2;  //(maxX + minX) x_nucleo accel calibration
//float accel_offsetY = (1989.00 + -1969.00) / 2;  //(maxY + minY)
//float accel_offsetZ = (1989.00 + -1988.00) / 2;  //(maxZ + minZ)

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);  //(cfg, encoder, RFID)
  M5.Display.setRotation(0);       //0-3 are clockwise rotations, 4-7 are counterclockwise rotations (installed @ 90 is 1)
  M5.Touch.setFlickThresh(50);     //flickThresh (distance)

  Serial.begin(115200);
  while (!Serial)
    ;

  DEV_I2C.begin();
  // Acc.begin();
  // Acc.Enable();
  Acc.DisableTemperatureSensor();
  Mag.begin();
  Mag.Enable();

  sx = M5Dial.Display.width() / 2;  //center of display
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);           //set max brightness
  M5Dial.Display.setTextDatum(middle_center);  //sx, sy

  PAGE();
  M5Dial.Display.drawString("ENL", sx, sy - 100);  //start-up page
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

    unsigned long timeRemaining = (mins * 60) + secs;

    if (timeRemaining <= 0) {
      isTimer_running = false;  // Stop the timer
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(BLUE, bkgrnd_color);
      M5Dial.Display.setTextFont(8);
      M5Dial.Display.drawString("00:00", sx, sy);
    }
  }
}

void COMPASS() {
  M5Dial.Display.setTextSize(2.25);  //compass digits
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);

  int32_t magnetometer[3];
  Mag.GetAxes(magnetometer);

  float mag_x = magnetometer[0];
  float mag_y = magnetometer[1];
  float mag_z = magnetometer[2];

  int cal_magX = mag_x - mag_offsetX;  //Apply mag calibration offsets
  int cal_magY = mag_y - mag_offsetY;
  int cal_magZ = mag_y - mag_offsetZ;

  if ((millis() - lastDisplayTime) > 1000) {               //print heading delay
    int heading = atan2(cal_magY, cal_magX) * (180 / PI);  // Convert to 0-360 degrees
    if (heading < 0) {
      heading += 360;
    }

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

void BEACON() {
  M5Dial.Display.setTextSize(2);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);
  M5Dial.Display.drawString("1202", sx, sy + 30);
}

void ENL_DISPLAY() {
  char dig_buf[10];        //buffer to store ENL digits
  char timer_buf[10];      //buffer for ENL time
  char threshold_buf[15];  //buffer for ENL_threshold display

  long newPosition = M5Dial.Encoder.read();  //adjust threshold
  if (newPosition > oldPosition) {
    oldPosition = newPosition;
    ENL_threshold++;
  }
  if (newPosition < oldPosition) {
    oldPosition = newPosition;
    ENL_threshold--;
  }
  ENL_threshold = constrain(ENL_threshold, 0, 100);

  snprintf(threshold_buf, sizeof(threshold_buf), "  THRESH: %-2i%%  ", ENL_threshold);  //print threshold for adjustment
  M5Dial.Display.drawString(threshold_buf, sx, sy - 35);

  snprintf(timer_buf, sizeof(timer_buf), " %-2dM %-2dS ", minutes, seconds);  //print ENL timer
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextFont(4);
  M5Dial.Display.setTextColor(GREEN, bkgrnd_color);
  M5Dial.Display.drawString(timer_buf, sx, sy - 75);

  if ((millis() - lastDisplayTime) > 1000) {                   //ENL display delay
    snprintf(dig_buf, sizeof(dig_buf), " %-2i%%", ENL_value);  //print ENL
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.setTextColor(WHITE, bkgrnd_color);
    M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);
    M5Dial.Display.drawString(dig_buf, sx, sy + 30);

    if (isENL_high) {
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
    lastDisplayTime = millis();
  }
}

void ENL_MEASURE() {
  int ENL_max = 0;
  int ENL_min = 4095;  //12 bit ADC

  if ((millis() - lastMeasureTime) > 1000) {  //ENL measurement delay
    for (int i = 0; i < 100; ++i) {           //Average samples
      ENL_value = map(analogRead(ENL_pin), 0, 4095, 0, 100);
      ENL_min = min(ENL_min, ENL_value);
      ENL_max = max(ENL_max, ENL_value);
    }
    ENL_value = (ENL_max - ENL_min);  //measure noise level

    lastMeasureTime = millis();
  }

  if (ENL_value > ENL_threshold) {
    isENL_high = true;             //set motor running bit to run
    int currentMillis = millis();  //Get the current time and increment counter

    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;
      seconds++;
      if (seconds >= 60) {
        seconds = 0;
        minutes++;
      }
    }
  } else {
    isENL_high = false;
  }

  if (isENL_high == false & isTimer_running == false & (((minutes * 60) + seconds) > last_runTime)) {  //test for 2nd, 3rd... new motor cool down time
    PAGE();
    M5Dial.Display.drawString("TIMER", sx, sy - 50);
    currentState = timer;                           //switch to timer screen
    isTimer_running = true;                         //set bit cool down timer
    startTime = millis();                           //restart timer
    last_runTime = ((minutes * 60) + seconds) + 5;  //check that it's a new motor run with 5 secs of debounce
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
    case enl_display:
      ENL_DISPLAY();
      break;
  }
}