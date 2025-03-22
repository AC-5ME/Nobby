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
  FUNCTION_THREE
};
State currentState = FUNCTION_ONE;  //set default screen to timer

void cool_down();  //function declarations
void compass();
void beacon();

time_t ttset = 0;
time_t ttold = 0;
int timer = 0;
int timeron = 0;  //timer off
int sx, sy;
int addtime = 300;  //default time 5 mins
int alarming = 0;   //alarm off

int col1 = WHITE;  //second arc

int timer_bkcolor = BLACK;  //background color
long oldPosition = -999;

float mag_offsetX = (64.00 + -1068.00) / 2;  //(maxX + minX) x_nucleo mag calibration
float mag_offsetY = (568.00 + -519.00) / 2;  //(maxY + minY)
float mag_offsetZ = (829.00 + -270.00) / 2;  //(maxZ + minZ)

float accel_offsetX = (1989.00 + -1988.00) / 2;  //(maxX + minX) x_nucleo accel calibration
float accel_offsetY = (1989.00 + -1969.00) / 2;  //(maxY + minY)
float accel_offsetZ = (1989.00 + -1988.00) / 2;  //(maxZ + minZ)

long lastDisplayTime;

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(0);  //0-3 are clockwise rotations, 4-7 are counterclockwise rotations (installed @ 90 is 1)

  M5.Touch.setFlickThresh(50);  //flickThresh = distance

  Serial.begin(115200);
  while (!Serial)
    ;

  DEV_I2C.begin();
  Acc.begin();
  Acc.Enable();
  Acc.EnableTemperatureSensor();
  Mag.begin();
  Mag.Enable();

  sx = M5Dial.Display.width() / 2;  //center of display
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);  //set max brightness

  M5Dial.Display.fillScreen(timer_bkcolor);
  M5Dial.Display.setTextDatum(middle_center);

  M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);  //Label
  M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.drawString("COOL DOWN", sx, sy - 40);
  M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);  //X and Y to X1 and Y1, width W, height H, color

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
      case FUNCTION_ONE:  //cool_down
        //Serial.println("Switching to Function Two.");
        M5Dial.Display.fillScreen(timer_bkcolor);  //reset screen
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.drawString("HEADING", sx, sy - 40);
        currentState = FUNCTION_TWO;
        break;

      case FUNCTION_TWO:  //compass
        //Serial.println("Switching to Function Three.");
        M5Dial.Display.fillScreen(timer_bkcolor);
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.drawString("BEACON", sx, sy - 40);
        currentState = FUNCTION_THREE;
        break;

      case FUNCTION_THREE:  //beacon
        //Serial.println("Switching to Function One.");
        M5Dial.Display.fillScreen(timer_bkcolor);
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(1);
        M5Dial.Display.drawString("COOL DOWN", sx, sy - 40);
        currentState = FUNCTION_ONE;
        break;
    }
  }

  switch (currentState) {  //Call the current function based on the state
    case FUNCTION_ONE:
      cool_down();
      break;

    case FUNCTION_TWO:
      compass();
      break;

    case FUNCTION_THREE:
      beacon();
      break;
  }
}

void cool_down() {
  M5Dial.update();

  int min;
  int sec;
  char buf[6];  //buffer to store timer digits

  if (M5Dial.BtnA.isPressed()) {  //Start or stop timer
    delay(200);

    if (alarming == 0 && timeron == 0) {
      alarming = 0;
      auto dt = M5Dial.Rtc.getDateTime();
      ttold = dt2tt(dt, 0);
      ttset = ttold + timer;
      timeron = 1;

      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);
    }

    else {
      alarming = 0;  //Stop timer
      timeron = 0;

      min = (timer / 60);
      sec = (timer % 60);

      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
      M5Dial.Display.setTextFont(8);
      sprintf(buf, "%02d:%02d", min, sec);
      M5Dial.Display.drawString(buf, sx, sy);

      drawarc(min, sec, 0);
    }
  }

  auto t = M5Dial.Touch.getDetail();

  if (t.state == 1 && timeron == 0) {  //1 = tap

    if (addtime == 1) {
      addtime = 60;
      M5Dial.Display.fillRect(sx + 25, sy + 40, 60, 4, timer_bkcolor);
      M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);
    } else {
      addtime = 1;
      M5Dial.Display.fillRect(sx + 25, sy + 40, 60, 4, WHITE);
      M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, timer_bkcolor);
    }
  }

  if (timeron == 0) {
    long newPosition = (M5Dial.Encoder.read() / 4);
    int redraw = 0;
    if (newPosition > oldPosition) {
      if (timer < 3599) {
        if ((timer + addtime) < 3600) {
          timer += addtime;
          redraw = 1;
        } else {
          timer = 3599;
          redraw = 1;
        }
      }
      oldPosition = newPosition;
    } else if (newPosition < oldPosition) {
      if (timer > 0) {
        if ((timer - addtime) >= 0) {
          timer -= addtime;
          redraw = 1;
        } else {
          timer = 0;
          redraw = 1;
        }
      }
      oldPosition = newPosition;
    }

    if (redraw != 0) {
      min = (timer / 60);
      sec = (timer % 60);
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
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

  if (tt > ttold && timeron != 0) {
    if ((ttset - tt) >= 0) {
      min = ((ttset - tt) / 60);
      sec = ((ttset - tt) % 60);
      M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
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

void compass() {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);

  //float temperature;
  //Acc.GetTemperature(&temperature);

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
      case 0 ... 30:
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(4);
        M5Dial.Display.drawString(" N ", sx, sy + 40);
        break;

      case 31 ... 135:
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(4);
        M5Dial.Display.drawString(" E ", sx, sy + 40);
        break;

      case 136 ... 230:
       M5Dial.Display.setTextSize(0.8);
      M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
      M5Dial.Display.setTextFont(8);
        M5Dial.Display.drawString(" S ", sx, sy);
        break;

      case 231 ... 300:
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(4);
        M5Dial.Display.drawString(" W ", sx, sy + 40);
        break;

      case 301 ... 360:
        M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);
        M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
        M5Dial.Display.setTextSize(4);
        M5Dial.Display.drawString(" N ", sx, sy + 40);
        break;
    }
    lastDisplayTime = millis();
  }
}

void beacon() {
  //Serial.println("Function Three is running...");

  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();
}