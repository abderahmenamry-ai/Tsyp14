// hal_esp32.cpp - Hal on real hardware (Arduino-ESP32 core 2.x, PlatformIO platform espressif32@^6).
// NOT compiled or run in the project's simulation environment: `make check-esp32` only syntax-checks it against
// stub headers. Bring-up checklist: firmware/README.md.
#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>
#include "hal_esp32.h"
#include "pins.h"

// ------------------------------------------------------------------ encoders (quadrature, ISR decoded)
static volatile int32_t g_encL = 0, g_encR = 0;
static volatile uint8_t g_stL = 0, g_stR = 0;
static const int8_t QEM[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

static void IRAM_ATTR isrLeft() {
  uint8_t cur = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  g_stL = (uint8_t)(((g_stL << 2) | cur) & 0x0F);
  g_encL += ENC_L_SIGN * QEM[g_stL];
}
static void IRAM_ATTR isrRight() {
  uint8_t cur = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
  g_stR = (uint8_t)(((g_stR << 2) | cur) & 0x0F);
  g_encR += ENC_R_SIGN * QEM[g_stR];
}

// ------------------------------------------------------------------ radio
static RF24 g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static const uint8_t PREFIX[4] = RF_ADDR_PREFIX;
static void addrOf(uint8_t node, uint8_t out[5]) {
  for (int i = 0; i < 4; i++) out[i] = PREFIX[i];
  out[4] = (node == NODE_HUB) ? 0x10 : node;
}

#define LEDC_FREQ 20000
#define LEDC_BITS 10

void Esp32Hal::begin(Role role) {
  role_ = role;
  self_ = role == ROLE_WRITER ? NODE_WRITER : (role == ROLE_EXECUTOR ? NODE_EXECUTOR : NODE_HUB);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(921600);                                   // host link (USB)

  if (role != ROLE_HUB) {
    // motors
    pinMode(PIN_MOT_L_IN1, OUTPUT); pinMode(PIN_MOT_L_IN2, OUTPUT);
    pinMode(PIN_MOT_R_IN1, OUTPUT); pinMode(PIN_MOT_R_IN2, OUTPUT);
    ledcSetup(0, LEDC_FREQ, LEDC_BITS); ledcAttachPin(PIN_MOT_L_PWM, 0);
    ledcSetup(1, LEDC_FREQ, LEDC_BITS); ledcAttachPin(PIN_MOT_R_PWM, 1);
    setMotor(LEFT, 0); setMotor(RIGHT, 0);
    // encoders
    pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrRight, CHANGE);
    // IMU: wake the MPU-6050, gyro range +-250 deg/s (131 LSB per deg/s)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); imuOk_ = (Wire.endTransmission() == 0);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();   // DLPF ~44 Hz
    // LiDAR
    Serial2.setRxBufferSize(4096);
    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, -1);
  }
  if (role == ROLE_WRITER) {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MQ2_AO, ADC_11db);
    pinMode(PIN_TILT_DO, INPUT_PULLUP);
    pinMode(PIN_PIR_OUT, INPUT_PULLDOWN);
  }

  // radio (all roles)
  radioOk_ = g_radio.begin();
  if (radioOk_) {
    g_radio.setChannel(RF_CHANNEL);
    g_radio.setDataRate(RF24_250KBPS);                    // best sensitivity = best range underground
    g_radio.setPALevel(RF24_PA_MAX);
    g_radio.setCRCLength(RF24_CRC_16);
    g_radio.setAutoAck(true);
    g_radio.enableDynamicPayloads();
    g_radio.setRetries(5, 15);                            // 1500 us between retries, up to 15
    uint8_t a[5]; addrOf(self_, a);
    g_radio.openReadingPipe(1, a);
    g_radio.startListening();
  }
}

uint32_t Esp32Hal::millis() { return ::millis(); }
uint32_t Esp32Hal::micros() { return ::micros(); }

void Esp32Hal::setMotor(Side s, float duty) {
  if (duty > 1) duty = 1;
  if (duty < -1) duty = -1;
  bool inv = (s == LEFT) ? MOTOR_L_INVERT : MOTOR_R_INVERT;
  if (inv) duty = -duty;
  int in1 = (s == LEFT) ? PIN_MOT_L_IN1 : PIN_MOT_R_IN1;
  int in2 = (s == LEFT) ? PIN_MOT_L_IN2 : PIN_MOT_R_IN2;
  digitalWrite(in1, duty > 0);
  digitalWrite(in2, duty < 0);
  uint32_t pwm = (uint32_t)(fabsf(duty) * ((1 << LEDC_BITS) - 1));
  ledcWrite(s == LEFT ? 0 : 1, pwm);
}

int32_t Esp32Hal::encoderCount(Side s) {
  noInterrupts();
  int32_t v = (s == LEFT) ? g_encL : g_encR;
  interrupts();
  return v;
}

bool Esp32Hal::readGyroZ(float *dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                                       // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return false;
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  *dps = raw / 131.0f;
  return true;
}

int Esp32Hal::lidarAvailable() { return Serial2.available(); }
int Esp32Hal::lidarRead() { return Serial2.read(); }
int Esp32Hal::hostAvailable() { return Serial.available(); }
int Esp32Hal::hostRead() { return Serial.read(); }
void Esp32Hal::hostWrite(const uint8_t *buf, size_t len) { Serial.write(buf, len); }

bool Esp32Hal::radioSend(uint8_t dst, const uint8_t *buf, uint8_t len) {
  if (!radioOk_) return false;
  uint8_t a[5]; addrOf(dst, a);
  g_radio.stopListening();
  g_radio.openWritingPipe(a);
  bool ok = g_radio.write(buf, len);                      // blocks until ACK or 15 retries
  lastRetries_ = g_radio.getARC();
  g_radio.startListening();
  return ok;
}

int Esp32Hal::radioRead(uint8_t *buf) {
  if (!radioOk_ || !g_radio.available()) return 0;
  uint8_t len = g_radio.getDynamicPayloadSize();
  if (len < 1 || len > 32) { g_radio.flush_rx(); return 0; }
  g_radio.read(buf, len);
  return len;
}

int Esp32Hal::readGasRaw() { return analogRead(PIN_MQ2_AO); }
bool Esp32Hal::readTilt() { return digitalRead(PIN_TILT_DO) == TILT_TRIPPED_LEVEL; }
bool Esp32Hal::readPir() { return digitalRead(PIN_PIR_OUT) == HIGH; }
uint16_t Esp32Hal::batteryMv() { return 0; }              // PIN_BATT_SENSE not wired (see pins.h)
void Esp32Hal::setLed(bool on) { digitalWrite(PIN_LED, on); }
