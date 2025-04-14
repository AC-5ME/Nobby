
#include <M5Dial.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>

#define DEV_I2C Wire
#define SerialPort Serial

LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

enum State {
  FUNCTION_ONE,
  FUNCTION_TWO,
  FUNCTION_THREE,
  FUNCTION_FOUR,
};
State currentState = FUNCTION_ONE;  //set default screen to TIMER

void TIMER();  //function declarations
void COMPASS();
void BEACON();
void ENL();

time_t ttset = 0;
time_t ttold = 0;
int time_r = 0;
int TIMERon = 0;    //TIMER off
int sx, sy;         //display center point
int addtime = 300;  //default time 5 mins
int alarming = 0;   //alarm off
int col1 = WHITE;   //second arc
int TIMER_bkcolor = BLACK;  //background color
long oldPosition = -999;

const int ENL_pin = 2;  //ENL pin, blue wire
int ENL_value = 0;
int ENL_loudness = 0;           //difference between baseline ADC reading and noise level
const int ENL_threshold = 300;  //motor running noise level?
int ENL_samples = 500;         //average ENL over time
bool motor_running = false;     //motor run TIMER
int counter = 0; // Initialize counter variable
unsigned long previousMillis = 0; // Store the last time the counter was updated
const long interval = 1000;

float mag_offsetX = (291 + -861) / 2;  //(maxX + minX) x_nucleo mag calibration
float mag_offsetY = (351 + -738) / 2;  //(maxY + minY)
float mag_offsetZ = (853 + -277) / 2;  //(maxZ + minZ)

//float accel_offsetX = (1989.00 + -1988.00) / 2;  //(maxX + minX) x_nucleo accel calibration
//float accel_offsetY = (1989.00 + -1969.00) / 2;  //(maxY + minY)
//float accel_offsetZ = (1989.00 + -1988.00) / 2;  //(maxZ + minZ)

long lastDisplayTime;  //delay function

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(0);    //0-3 are clockwise rotations, 4-7 are counterclockwise rotations (installed @ 90 is 1)
  M5.Touch.setFlickThresh(50);  //flickThresh = distance

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

  M5Dial.Display.setBrightness(255);  //set max brightness

  M5Dial.Display.fillScreen(TIMER_bkcolor);
  M5Dial.Display.setTextDatum(middle_center);

  auto dt = M5Dial.Rtc.getDateTime();
  ttset = dt2tt(dt, 1);
  ttold = ttset;
}

time_t dt2tt(m5::rtc_datetime_t dt, int pp) {  //Set RTC
  struct tm tm;
  tm.tm_year = dt.date.year - 1900;
  tm.tm_mon = dt.date.month - 1;
  tm.tm_mday = dt.date.date;
  tm.tm_wday = dt.date.weekDay;
  tm.tm_hour = dt.time.hours;
  tm.tm_min = dt.time.minutes;
  tm.tm_sec = dt.time.seconds;
  tm.tm_isdst = -1;

  time_t tt = mktime(&tm) + (8 * 60 * 60);
  return (tt);
}

void loop() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  if (t.state == 9) {  //swipe?
    delay(50);         //debounce swipe
    switch (currentState) {
      case FUNCTION_ONE:  //TIMER
        //Serial.println("Switching to Function Two.");
        M5Dial.Display.fillScreen(TIMER_bkcolor);  //reset screen
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
        M5Dial.Display.setTextFont(4);
        M5Dial.Display.drawString("HEADING", sx, sy - 50);
        currentState = FUNCTION_TWO;
        break;

      case FUNCTION_TWO:  //COMPASS
        //Serial.println("Switching to Function Three.");
        M5Dial.Display.fillScreen(TIMER_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
        M5Dial.Display.setTextFont(4);
        M5Dial.Display.drawString("BEACON", sx, sy - 50);
        currentState = FUNCTION_THREE;
        break;

      case FUNCTION_THREE:  //BEACON
        //Serial.println("Switching to Function Four.");
        M5Dial.Display.fillScreen(TIMER_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
        M5Dial.Display.setTextFont(4);
        M5Dial.Display.drawString("ENL", sx, sy - 50);
        currentState = FUNCTION_FOUR;
        break;

      case FUNCTION_FOUR:  //vib
        //Serial.println("Switching to Function One.");
        M5Dial.Display.fillScreen(TIMER_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
        M5Dial.Display.setTextFont(4);
        M5Dial.Display.drawString("TIMER", sx, sy - 50);
        currentState = FUNCTION_ONE;
        break;
    }
  }

  switch (currentState) {  //Call the current function based on the state
    case FUNCTION_ONE:
      TIMER();
      break;

    case FUNCTION_TWO:
      COMPASS();
      break;

    case FUNCTION_THREE:
      BEACON();
      break;

    case FUNCTION_FOUR:
      ENL();
      break;
  }
}

void TIMER() {
  M5Dial.update();

  int min;
  int sec;
  char buf[6];  //buffer to store TIMER digits

  M5Dial.Display.setTextSize(1);
  M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
  M5Dial.Display.setTextFont(4);
  M5Dial.Display.drawString("TIMER", sx, sy - 50);
  //M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);  //X and Y to X1 and Y1, width W, height H, color

  if (M5Dial.BtnA.isPressed()) {  //Start or stop TIMER
    delay(200);

    if (alarming == 0 && TIMERon == 0) {
      alarming = 0;
      auto dt = M5Dial.Rtc.getDateTime();
      ttold = dt2tt(dt, 0);
      ttset = ttold + time_r;
      TIMERon = 1;

      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);
    }

    else {
      alarming = 0;  //Stop TIMER
      TIMERon = 0;

      min = (time_r / 60);
      sec = (time_r % 60);

      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);

      drawarc(min, sec, 0);
    }
  }

  auto t = M5Dial.Touch.getDetail();

  if (t.state == 1 && TIMERon == 0) {  //1 = tap

    if (addtime == 1) {
      addtime = 60;
      M5Dial.Display.fillRect(sx + 25, sy + 40, 60, 4, TIMER_bkcolor);
      M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);
    } else {
      addtime = 1;
      M5Dial.Display.fillRect(sx + 25, sy + 40, 60, 4, WHITE);
      M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, TIMER_bkcolor);
    }
  }

  if (TIMERon == 0) {
    long newPosition = (M5Dial.Encoder.read() / 4);
    int redraw = 0;
    if (newPosition > oldPosition) {
      if (time_r < 3599) {
        if ((time_r + addtime) < 3600) {
          time_r += addtime;
          redraw = 1;
        } else {
          time_r = 3599;
          redraw = 1;
        }
      }
      oldPosition = newPosition;
    } else if (newPosition < oldPosition) {
      if (time_r > 0) {
        if ((time_r - addtime) >= 0) {
          time_r -= addtime;
          redraw = 1;
        } else {
          time_r = 0;
          redraw = 1;
        }
      }
      oldPosition = newPosition;
    }

    if (redraw != 0) {
      min = (time_r / 60);
      sec = (time_r % 60);
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);

      drawarc(min, sec, 0);
    }
  }

  auto dt = M5Dial.Rtc.getDateTime();
  time_t tt = dt2tt(dt, 0);

  if (tt != ttold) {
    if (tt > 2000000000) {
      tt = ttold;
    } else if (tt < 0) {
      tt = ttold;
    } else if (((tt - ttold) > 60) || ((tt - ttold) < -60)) {
      tt = ttold;
    }
  }

  if (tt > ttold && TIMERon != 0) {
    if ((ttset - tt) >= 0) {
      min = ((ttset - tt) / 60);
      sec = ((ttset - tt) % 60);
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);

      drawarc(min, sec, 1);
    } else {
      if (alarming == 0) {
      }
      alarming = 1;
      //Alarm?
    }
    ttold = tt;
  }
}

void drawarc(int min, int sec, int mode) {

  int rmin = min * 6;
  int rsec = sec * 6;

  if (rsec < 90) {
    M5Dial.Display.fillArc(sx, sy, 102, 110, 270, 270 + rsec, col1);
    M5Dial.Display.fillArc(sx, sy, 102, 110, 270 + rsec, 360, BLACK);
    M5Dial.Display.fillArc(sx, sy, 102, 110, 0, 270, BLACK);
  } else {
    M5Dial.Display.fillArc(sx, sy, 102, 110, 270, 360, col1);
    M5Dial.Display.fillArc(sx, sy, 102, 110, 0, rsec - 90, col1);
    M5Dial.Display.fillArc(sx, sy, 102, 110, rsec - 90, 270, BLACK);
  }
}

void COMPASS() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  M5Dial.Display.setTextSize(2.25);
  M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);

  int32_t magnetometer[3];
  Mag.GetAxes(magnetometer);

  float mag_x = magnetometer[0];
  float mag_y = magnetometer[1];
  float mag_z = magnetometer[2];

  int cal_magX = mag_x - mag_offsetX;  //Apply mag calibration offsets
  int cal_magY = mag_y - mag_offsetY;
  int cal_magZ = mag_y - mag_offsetZ;

  if ((millis() - lastDisplayTime) > 500) {  //print heading delay

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
  //Serial.println("Function Three is running...");

  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();
}

void ENL() {
  char buf[4];  //buffer to store ENL digits
  static int ENL_last = 0;
  int ENL_max = 0;

  M5Dial.Display.setTextSize(2.25);
  M5Dial.Display.setTextColor(WHITE, TIMER_bkcolor);
  M5Dial.Display.setTextFont(&fonts::FreeSansBold24pt7b);

  if ((millis() - lastDisplayTime) > 250) {  //get ENL delay

    for (int i = 0; i < ENL_samples; i++) {
      ENL_value = analogRead(ENL_pin);
      ENL_loudness = abs(ENL_value - ENL_last);  //measurement above baseline ADC reading
      if (ENL_loudness > ENL_max) { ENL_max = ENL_loudness; }
      ENL_last = ENL_value;
    }

    snprintf(buf, sizeof(buf), "%-4d", ENL_max);
    M5Dial.Display.drawString(buf, sx, sy + 30);

    if (ENL_max > ENL_threshold) {
      unsigned long currentMillis = millis(); // Get the current time

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time the counter was updated
    counter++; // Increment the counter
    Serial.println(counter);
  }

      M5Dial.Display.setTextSize(1);
      M5Dial.Display.setTextColor(GREEN, TIMER_bkcolor);
      M5Dial.Display.setTextFont(4);
      M5Dial.Display.drawString("MOTOR RUNNING", sx, sy - 50);
    }

    lastDisplayTime = millis();
  }
}