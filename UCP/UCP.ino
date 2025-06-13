#define PIN_SPI_SS 10

#include <M5Dial.h>
#include <Appnostic_SC16IS752.h>
#include "uvc_arduino.h"

// --- UART and App State ---
#define UART_CHANNEL SC16IS752_CHANNEL_A
Appnostic_SC16IS752 uart(UART_CHANNEL);

// Squawk state (octal digits)
int squawk[4] = { 1, 2, 0, 2 };
int digit = 0;

// Transponder modes
const char *modes[] = { "STBY", "ON", "ALT" };
int modeIdx = 0;
bool showIdent = false;
unsigned long identDisplayUntil = 0;
unsigned long lastStatusRequest = 0;
int lastEncDetent = 0;


// --- Utility: Pack squawk into BCD ---
void squawkToBcd(uint8_t &high, uint8_t &low) {
  high = (squawk[0] << 4) | squawk[1];
  low = (squawk[2] << 4) | squawk[3];
}

// --- Draw UI with per-feature sprites ---
void drawUI() {
  static LGFX_Sprite modeSprite(&M5Dial.Display);
  static LGFX_Sprite squawkSprite(&M5Dial.Display);
  static LGFX_Sprite msgSprite(&M5Dial.Display);
  static bool initialized = false;

  if (!initialized) {
    modeSprite.createSprite(240, 50);     // Mode area (X, Y)
    squawkSprite.createSprite(240, 140);  // Squawk area
    msgSprite.createSprite(240, 40);    // msguctions area
    modeSprite.setTextDatum(middle_center);
    squawkSprite.setTextDatum(middle_center);
    msgSprite.setTextDatum(middle_center);
    initialized = true;
  }

  // --- Mode Sprite ---
  char mode_buf[10];

  modeSprite.fillSprite(BLACK);
  modeSprite.setTextFont(&fonts::Font4);
  modeSprite.setTextColor(GREEN, BLACK);

  snprintf(mode_buf, sizeof(mode_buf), "%s", modes[modeIdx]);

  modeSprite.drawCenterString(mode_buf, 120, 25);  // Center of createSprite field
  modeSprite.pushSprite(0, 0);

  // --- Squawk Sprite ---
  char digit_buf[8];
  int digit_count = 4;
  int sprite_width = 240;

  squawkSprite.fillSprite(BLACK);
  squawkSprite.setTextFont(&fonts::Font8);

  for (int i = 0; i < digit_count; i++) {
    snprintf(digit_buf, sizeof(digit_buf), "%d", squawk[i]);
    if (i == digit)
      squawkSprite.setTextColor(YELLOW, BLACK);
    else
      squawkSprite.setTextColor(WHITE, BLACK);
    // Evenly spaced centers: (sprite_width * (2*i+1))/(2*digit_count)
    int cx = sprite_width * (2 * i + 1) / (2 * digit_count);
    squawkSprite.drawCenterString(digit_buf, cx, 30);
  }
  squawkSprite.pushSprite(0, 50);  // (X, Y)

  // message sprite
  if (showIdent) {
    msgSprite.fillSprite(BLACK);
    msgSprite.setTextFont(&fonts::Font4);
    msgSprite.setTextColor(YELLOW, BLACK);
    msgSprite.drawCenterString("IDENT", 120, 20);
    msgSprite.pushSprite(0, 170);
  } else {
    msgSprite.fillSprite(BLACK);
    msgSprite.setTextFont(2);
    msgSprite.setTextColor(CYAN, BLACK);
    msgSprite.drawCenterString("BUTTON -> IDENT", 120, 20);
    msgSprite.pushSprite(0, 170);
  }
}

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5Dial.Display.setBrightness(255);

  Serial.begin(115200);

  // Use I2C
  if (!uart.begin_i2c()) {
    Serial.println("I2C not found");
  }

  uart.setFIFO(true);  // enable fifo
  uart.setBaudrate(57600);
  uart.setLine(8, 0, 1);  // 8,n,1
  requestStatus();

  lastEncDetent = M5Dial.Encoder.read() / 4;
}

void loop() {
  M5Dial.update();

  handleModeTouch();  //Tap to change
  handleTouchSelectDigit();

  // Rotary: Change value of selected digit (one notch = one increment)
  int encRaw = M5Dial.Encoder.read();
  int encDetent = encRaw / 4;  // each detent = 4 steps

  if (encDetent != lastEncDetent) {
    if (encDetent > lastEncDetent) {
      squawk[digit] = (squawk[digit] + 1) % 8;
      sendSetSquawk();
    } else if (encDetent < lastEncDetent) {
      squawk[digit] = (squawk[digit] + 7) % 8;
      sendSetSquawk();
    }
    lastEncDetent = encDetent;
  }

  // IDENT on long press
  if (M5Dial.BtnA.pressedFor(1000) && M5Dial.BtnA.isPressed()) {
    sendIdent();
    showIdent = true;
    identDisplayUntil = millis() + 2000;  // Show for 2 secs
  }

  // Turn off showIdent after timeout
  if (showIdent && millis() > identDisplayUntil) {
    showIdent = false;
  }

  if (millis() - lastStatusRequest > 5000) {
    requestStatus();
    lastStatusRequest = millis();
  }

  drawUI();
}

void handleModeTouch() {    //tap to change mode
  auto t = M5Dial.Touch.getDetail();
  if (t.wasPressed()) {
    // Adjust x/y ranges to fit your UI's mode display area
    if (t.y > 0 && t.y < 50 && t.x > 0 && t.x < 240) {
      modeIdx = (modeIdx + 1) % 3;
      sendSetMode();
    }
  }
}

void handleTouchSelectDigit() {   //tap to selct sqwuak digits
  auto t = M5Dial.Touch.getDetail();
  if (t.wasPressed()) {  // True on touch down event
    for (int i = 0; i < 4; i++) {
      int cx = 240 * (2 * i + 1) / 8;  // centers at 30, 90, 150, 210
      int min_x = cx - 30;
      int max_x = cx + 30;
      if (t.x > min_x && t.x < max_x && t.y > 48 && t.y < 168) {
        digit = i;
      }
    }
  }
}

// Compose and send a UCP frame using uvc_arduino.h struct and CRC
void sendUcpFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen) {
  ucp_frame_t frame;
  frame.start = UCP_SOM;
  frame.len = payloadLen + 1;  // type + payload
  frame.type = type;
  memset(frame.payload, 0, UCP_MAX_PAYLOAD);
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(frame.payload, payload, payloadLen);
  }
  frame.crc = ucp_crc16_ccitt(&frame.type, frame.len);
  frame.end = UCP_EOM;

  // Send the frame: start, len, type, payload, crc (2B), end
  int totalLen = 1 + 1 + 1 + payloadLen + 2 + 1;  // start, len, type, payload, crc, end
  uart.write((uint8_t *)&frame, totalLen);
}

void sendSetSquawk() {
  uint8_t high, low;
  squawkToBcd(high, low);
  uint8_t payload[] = { high, low };
  sendUcpFrame(0x10, payload, 2);
}

void sendIdent() {
  sendUcpFrame(0x12, nullptr, 0);
}

void sendSetMode() {
  uint8_t payload[] = { (uint8_t)modeIdx };
  sendUcpFrame(0x11, payload, 1);
}

void requestStatus() {
  sendUcpFrame(0x80, nullptr, 0);
}

// Parse incoming UCP input and update state
void parseUcpInput() {
  // Wait for SOM
  while (uart.available() && uart.read() != UCP_SOM)
    ;
  if (uart.available() < 2) return;
  uint8_t len = uart.read();
  if (uart.available() < len + 3) return;  // type+payload+crc+eom

  uint8_t type = uart.read();
  uint8_t payload[UCP_MAX_PAYLOAD] = { 0 };
  for (int i = 0; i < len - 1; i++) payload[i] = uart.read();
  uint8_t crc_lo = uart.read();
  uint8_t crc_hi = uart.read();
  uint8_t eom = uart.read();

  uint8_t checkArr[UCP_MAX_PAYLOAD + 1] = { 0 };
  checkArr[0] = type;
  for (int i = 0; i < len - 1; i++) checkArr[i + 1] = payload[i];
  uint16_t calc_crc = ucp_crc16_ccitt(checkArr, len);

  uint16_t rx_crc = crc_lo | (crc_hi << 8);
  if (calc_crc != rx_crc) return;

  if (type == 0x81) {  // Status
    modeIdx = payload[0];
    squawk[0] = (payload[1] >> 4) & 0x0F;
    squawk[1] = payload[1] & 0x0F;
    squawk[2] = (payload[2] >> 4) & 0x0F;
    squawk[3] = payload[2] & 0x0F;
  }
}