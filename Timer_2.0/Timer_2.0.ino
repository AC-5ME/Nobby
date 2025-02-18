#include <M5Dial.h>

static m5::touch_state_t prev_state;  //test loop
int prev_x = -1;
int prev_y = -1;

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
  //Serial.begin(115200);

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


time_t dt2tt(m5::rtc_datetime_t dt, int pp) {
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

  auto t = M5Dial.Touch.getDetail();  //touch detected?

  if (t.state == 1 && timeron == 0) {  //tap

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

  if (t.state == 9) {  //9 = flick, 13 = drag.  Swipe to next screen
    M5Dial.Display.fillScreen(timer_bkcolor);

    test();
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
      //M5Dial.Speaker.tone(4000, 100);
      //delay(100);
      //M5Dial.Speaker.tone(4000, 100);
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

void test(void) {
  M5Dial.update();
  auto t = M5Dial.Touch.getDetail();  //touch detected?

  while (t.state == 0) {

    if (t.state == 9) {
      return;
    }

    if (prev_state != t.state) {
      prev_state = t.state;
      static constexpr const char* state_name[16] = {
        "none", "touch", "touch_end", "touch_begin",
        "___", "hold", "hold_end", "hold_begin",
        "___", "flick", "flick_end", "flick_begin",
        "___", "drag", "drag_end", "drag_begin"
      };
      M5_LOGI("%s", state_name[t.state]);
      M5Dial.Display.fillRect(0, 0, M5Dial.Display.width(),
                              M5Dial.Display.height() / 2, BLACK);

      M5Dial.Display.drawString(state_name[t.state],
                                M5Dial.Display.width() / 2,
                                M5Dial.Display.height() / 2 - 30);
    }
    if (prev_x != t.x || prev_y != t.y) {
      M5Dial.Display.fillRect(0, M5Dial.Display.height() / 2,
                              M5Dial.Display.width(),
                              M5Dial.Display.height() / 2, BLACK);
      M5Dial.Display.drawString(
        "X:" + String(t.x) + " / " + "Y:" + String(t.y),
        M5Dial.Display.width() / 2, M5Dial.Display.height() / 2 + 30);
      prev_x = t.x;
      prev_y = t.y;
      M5Dial.Display.drawPixel(prev_x, prev_y);
    }
  }
}