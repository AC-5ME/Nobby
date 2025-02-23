#include <M5Dial.h>

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
int timeron = 0;
int sx, sy;
int addtime = 300;  //default time 5 mins
int alarming = 0;

int col1 = WHITE;  //second arc

int timer_bkcolor = BLACK;  //background color
long oldPosition = -999;

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  Serial.begin(115200);
  while (!Serial)
    ;

  sx = M5Dial.Display.width() / 2;  //center of display
  sy = M5Dial.Display.height() / 2;

  M5Dial.Display.setBrightness(255);  //set max brightness

  M5Dial.Display.fillScreen(timer_bkcolor);
  M5Dial.Display.setTextDatum(middle_center);

  M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);  //Label
  M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
  M5Dial.Display.setTextSize(0.8);
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

  if (t.state == 11) {  //11 = flick, 9 = drag.  Swipe to next screen
    switch (currentState) {
      case FUNCTION_ONE:  //cool_down
        Serial.println("Switching to Function Two.");
        M5Dial.Display.fillScreen(timer_bkcolor);
        currentState = FUNCTION_TWO;
        break;

      case FUNCTION_TWO:  //compass
        Serial.println("Switching to Function Three.");
        M5Dial.Display.fillScreen(timer_bkcolor);
        currentState = FUNCTION_THREE;
        break;

      case FUNCTION_THREE:  //beacon
        Serial.println("Switching to Function One.");
        M5Dial.Display.fillScreen(timer_bkcolor);
        currentState = FUNCTION_ONE;
        break;
    }
  }
    //delay(200);  //debounce?

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
  auto t = M5Dial.Touch.getDetail();
  Serial.println("Function one is running...");

  int min;
  int sec;
  char buf[100];

  if (M5Dial.BtnA.wasPressed()) {  //Start timer
    if (alarming == 0 && timeron == 0) {
      alarming = 0;
      auto dt = M5Dial.Rtc.getDateTime();
      ttold = dt2tt(dt, 0);
      ttset = ttold + timer;
      timeron = 1;
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
      //Serial.println(buf);

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
  Serial.println("Function Two is running...");

  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);  //Label
  M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.drawString("COMPASS", sx, sy - 40);
  M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);  //X and Y to X1 and Y1, width W, height H, color
}


void beacon() {
  Serial.println("Function Three is running...");

  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();

  M5Dial.Display.setTextFont(&fonts::FreeSansBold12pt7b);  //Label
  M5Dial.Display.setTextColor(WHITE, timer_bkcolor);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.drawString("BEACON", sx, sy - 40);
  M5Dial.Display.fillRect(sx - 80, sy + 40, 60, 4, WHITE);  //X and Y to X1 and Y1, width W, height H, color
}
