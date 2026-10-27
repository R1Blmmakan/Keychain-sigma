#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

#define I2C_SDA 8
#define I2C_SCL 9

const int LEFT_EYE_CX = 38;
const int RIGHT_EYE_CX = 90;
const int EYE_CY = 28;
const int BASE_EYE_W = 26;
const int BASE_EYE_H = 34;
const int EYE_RADIUS = 8;

const float MAX_LOOK_X = 16.0f;
const float MAX_LOOK_Y = 8.0f;
const float SENSITIVITY_X = 2.4f;
const float SENSITIVITY_Y = 1.6f;
const float SMOOTHING_FACTOR = 0.20f;
const float SENSOR_SMOOTHING = 0.16f;
const float DEADZONE_ACCEL = 0.65f;
const float DIZZY_GYRO_THRESHOLD_RAD = 3.0f;
const unsigned long DIZZY_SUSTAINED_MS = 200;

const unsigned long FRAME_INTERVAL_MS = 33;
const unsigned long BLINK_DURATION_MS = 140;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t mpuAddress = 0;
bool mpuAvailable = false;
unsigned long lastFrameTime = 0;
unsigned long nextBlinkTime = 2600;
unsigned long blinkEndTime = 0;
unsigned long dizzyUntil = 0;
unsigned long shockedUntil = 0;
unsigned long dizzyStreakMs = 0;

float baseAx = 0.0f;
float baseAy = 0.0f;
float filteredAx = 0.0f;
float filteredAy = 0.0f;

float currentLookX = 0.0f;
float currentLookY = 0.0f;
float sleepyFactor = 0.0f;

float prevAccelX = 0.0f;
float prevAccelY = 0.0f;
float prevAccelZ = 9.8f;

void setup() {
  delay(150);

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  bool displayReady = false;
  for (uint8_t retry = 0; retry < 5; retry++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
      displayReady = true;
      break;
    }
    delay(50);
  }

  if (!displayReady) {
    return;
  }

  display.clearDisplay();

  uint8_t scanResults[8];
  uint8_t scanCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == OLED_ADDRESS) {
      continue;
    }
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (scanCount < 8) {
        scanResults[scanCount++] = addr;
      }
      if (addr == 0x68 || addr == 0x69) {
        mpuAddress = addr;
      }
    }
  }

  if (mpuAddress == 0 && scanCount > 0) {
    mpuAddress = scanResults[0];
  }

  if (mpuAddress != 0) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(mpuAddress);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(22, 28);
    display.println("CALIBRATING...");
    display.display();

    float sumAx = 0.0f;
    float sumAy = 0.0f;
    int samples = 0;

    for (int i = 0; i < 40; i++) {
      Wire.beginTransmission(mpuAddress);
      Wire.write(0x3B);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)mpuAddress, 6) == 6) {
        int16_t rx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t ry = (int16_t)((Wire.read() << 8) | Wire.read());
        Wire.read();
        Wire.read();
        sumAx += ((float)rx / 16384.0f) * 9.80665f;
        sumAy += ((float)ry / 16384.0f) * 9.80665f;
        samples++;
      }
      delay(8);
    }

    if (samples > 0) {
      baseAx = sumAx / (float)samples;
      baseAy = sumAy / (float)samples;
    }

    mpuAvailable = true;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastFrameTime < FRAME_INTERVAL_MS) {
    return;
  }
  lastFrameTime = currentMillis;

  float targetLookX = 0.0f;
  float targetLookY = 0.0f;
  float targetSleepy = 0.0f;

  if (mpuAvailable) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0) {
      if (Wire.requestFrom((int)mpuAddress, 14) == 14) {
        int16_t rawAx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawAy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawAz = (int16_t)((Wire.read() << 8) | Wire.read());
        Wire.read();
        Wire.read();
        int16_t rawGx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGz = (int16_t)((Wire.read() << 8) | Wire.read());

        float ax = ((float)rawAx / 16384.0f) * 9.80665f;
        float ay = ((float)rawAy / 16384.0f) * 9.80665f;
        float az = ((float)rawAz / 16384.0f) * 9.80665f;

        float gx = (float)rawGx / 131.0f;
        float gy = (float)rawGy / 131.0f;
        float gz = (float)rawGz / 131.0f;
        float gyroMagDeg = sqrt(gx * gx + gy * gy + gz * gz);
        float gyroMagRad = gyroMagDeg * 0.01745329f;

        float deltaX = ax - prevAccelX;
        float deltaY = ay - prevAccelY;
        float deltaZ = az - prevAccelZ;
        float deltaMotion = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        float totalAccel = sqrt(ax * ax + ay * ay + az * az);

        prevAccelX = ax;
        prevAccelY = ay;
        prevAccelZ = az;

        bool isUpsideDown = (az < -5.5f) || (ay > 7.0f);

        if (currentMillis > 800) {
          if (isUpsideDown || deltaMotion > 13.0f || totalAccel > 20.0f) {
            shockedUntil = currentMillis + 1400;
          } else if (gyroMagRad > DIZZY_GYRO_THRESHOLD_RAD) {
            dizzyStreakMs += FRAME_INTERVAL_MS;
            if (dizzyStreakMs >= DIZZY_SUSTAINED_MS) {
              if (currentMillis >= shockedUntil) {
                dizzyUntil = currentMillis + 2000;
              }
            }
          } else {
            if (dizzyStreakMs > FRAME_INTERVAL_MS) {
              dizzyStreakMs -= FRAME_INTERVAL_MS;
            } else {
              dizzyStreakMs = 0;
            }
          }
        }

        float netAx = ax - baseAx;
        float netAy = ay - baseAy;

        if (fabs(netAx) < DEADZONE_ACCEL) {
          netAx = 0.0f;
        } else if (netAx > 0.0f) {
          netAx -= DEADZONE_ACCEL;
        } else {
          netAx += DEADZONE_ACCEL;
        }

        if (fabs(netAy) < DEADZONE_ACCEL) {
          netAy = 0.0f;
        } else if (netAy > 0.0f) {
          netAy -= DEADZONE_ACCEL;
        } else {
          netAy += DEADZONE_ACCEL;
        }

        filteredAx += (netAx - filteredAx) * SENSOR_SMOOTHING;
        filteredAy += (netAy - filteredAy) * SENSOR_SMOOTHING;

        if (filteredAy < -2.4f && !isUpsideDown) {
          targetSleepy = 1.0f;
        }

        targetLookX = -filteredAx * SENSITIVITY_X;
        targetLookY = filteredAy * SENSITIVITY_Y;

        if (targetLookX > MAX_LOOK_X) {
          targetLookX = MAX_LOOK_X;
        } else if (targetLookX < -MAX_LOOK_X) {
          targetLookX = -MAX_LOOK_X;
        }

        if (targetLookY > MAX_LOOK_Y) {
          targetLookY = MAX_LOOK_Y;
        } else if (targetLookY < -MAX_LOOK_Y) {
          targetLookY = -MAX_LOOK_Y;
        }
      }
    }
  }

  currentLookX += (targetLookX - currentLookX) * SMOOTHING_FACTOR;
  currentLookY += (targetLookY - currentLookY) * SMOOTHING_FACTOR;
  sleepyFactor += (targetSleepy - sleepyFactor) * 0.16f;

  bool isShocked = (currentMillis < shockedUntil);
  bool isDizzy = (!isShocked && currentMillis < dizzyUntil);
  bool isSleepy = (!isShocked && !isDizzy && sleepyFactor > 0.05f);

  float blinkScale = 0.0f;
  if (!isShocked && !isDizzy && !isSleepy) {
    if (currentMillis >= nextBlinkTime && currentMillis >= blinkEndTime) {
      blinkEndTime = currentMillis + BLINK_DURATION_MS;
      nextBlinkTime = currentMillis + (unsigned long)random(2400, 4600);
    }
    if (currentMillis < blinkEndTime) {
      float progress = (float)(BLINK_DURATION_MS - (blinkEndTime - currentMillis)) / (float)BLINK_DURATION_MS;
      blinkScale = sin(progress * 3.14159f);
    }
  }

  display.clearDisplay();

  if (isShocked) {
    for (int i = 0; i < 2; i++) {
      int cx = (i == 0) ? LEFT_EYE_CX : RIGHT_EYE_CX;
      int cy = EYE_CY;
      display.fillRoundRect(cx - 11, cy - 19, 22, 38, 11, SSD1306_WHITE);
      display.fillRect(cx - 2, cy - 2, 4, 4, SSD1306_BLACK);
    }

    display.fillTriangle(114, 14, 111, 20, 117, 20, SSD1306_WHITE);
    display.fillCircle(114, 21, 3, SSD1306_WHITE);

    display.fillRoundRect(60, 47, 8, 11, 4, SSD1306_WHITE);
    display.fillRoundRect(61, 48, 6, 9, 3, SSD1306_BLACK);
  } else if (isDizzy) {
    float spin = (float)currentMillis * 0.012f;

    for (int i = 0; i < 2; i++) {
      int cx = (i == 0) ? (LEFT_EYE_CX + (int)round(currentLookX * 0.5f)) : (RIGHT_EYE_CX + (int)round(currentLookX * 0.5f));
      int cy = EYE_CY + (int)round(currentLookY * 0.5f);

      float prevX = cx;
      float prevY = cy;

      for (int step = 1; step <= 36; step++) {
        float t = (float)step / 36.0f;
        float r = 1.0f + t * 13.0f;
        float theta = t * 4.0f * 3.14159f + spin;
        float curX = cx + cos(theta) * r;
        float curY = cy + sin(theta) * r;
        display.drawLine((int16_t)round(prevX), (int16_t)round(prevY), (int16_t)round(curX), (int16_t)round(curY), SSD1306_WHITE);
        display.drawLine((int16_t)round(prevX) + 1, (int16_t)round(prevY), (int16_t)round(curX) + 1, (int16_t)round(curY), SSD1306_WHITE);
        prevX = curX;
        prevY = curY;
      }
    }

    for (int x = 56; x <= 72; x++) {
      int my = 51 + (int)round(sin((float)(x - 56) * 0.6f + spin * 0.5f) * 2.5f);
      display.drawPixel(x, my, SSD1306_WHITE);
      display.drawPixel(x, my + 1, SSD1306_WHITE);
    }
  } else {
    float idleBob = sin((float)currentMillis * 0.003f) * 1.5f;
    int currentEyeH = (int)(BASE_EYE_H * (1.0f - blinkScale * 0.92f));
    if (currentEyeH < 2) {
      currentEyeH = 2;
    }

    int r = EYE_RADIUS;
    if (r > currentEyeH / 2) {
      r = currentEyeH / 2;
    }

    for (int i = 0; i < 2; i++) {
      int cx = (i == 0) ? (LEFT_EYE_CX + (int)round(currentLookX)) : (RIGHT_EYE_CX + (int)round(currentLookX));
      int cy = EYE_CY + (int)round(currentLookY + idleBob);

      int x0 = cx - BASE_EYE_W / 2;
      int y0 = cy - currentEyeH / 2;

      display.fillRoundRect(x0, y0, BASE_EYE_W, currentEyeH, r, SSD1306_WHITE);

      if (currentEyeH >= 10) {
        int pw = 14;
        int ph = (int)(20.0f * (float)currentEyeH / (float)BASE_EYE_H);
        int px0 = cx - pw / 2 + (int)round(currentLookX * 0.35f);
        int py0 = cy - ph / 2 + (int)round(currentLookY * 0.35f);
        int pr = 4;
        if (pr > ph / 2) {
          pr = ph / 2;
        }

        display.fillRoundRect(px0, py0, pw, ph, pr, SSD1306_BLACK);

        display.fillRect(px0 + 2, py0 + 2, 4, 3, SSD1306_WHITE);
        if (ph > 10) {
          display.fillRect(px0 + pw - 4, py0 + ph - 4, 2, 2, SSD1306_WHITE);
        }
      }

      if (isSleepy && currentEyeH > 6) {
        int droopH = (int)(currentEyeH * sleepyFactor * 0.72f);
        if (droopH > 0) {
          display.fillRect(x0 - 1, y0 - 1, BASE_EYE_W + 2, droopH, SSD1306_BLACK);
          display.drawFastHLine(x0, y0 + droopH - 1, BASE_EYE_W, SSD1306_WHITE);
        }
      }
    }

    int cheekLeftX = LEFT_EYE_CX - 12 + (int)round(currentLookX * 0.25f);
    int cheekRightX = RIGHT_EYE_CX + 8 + (int)round(currentLookX * 0.25f);
    int cheekY = 50 + (int)round(currentLookY * 0.25f + idleBob);
    display.fillRoundRect(cheekLeftX, cheekY, 6, 2, 1, SSD1306_WHITE);
    display.fillRoundRect(cheekRightX, cheekY, 6, 2, 1, SSD1306_WHITE);

    if (sleepyFactor > 0.4f) {
      int zY = (int)(15.0f - sin((float)currentMillis * 0.003f) * 3.0f);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(102, zY + 4);
      display.print("z");
      display.setCursor(112, zY);
      display.print("Z");
    }
  }

  if (!mpuAvailable) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(38, 56);
    display.print("NO SENSOR");
  }

  display.display();
}