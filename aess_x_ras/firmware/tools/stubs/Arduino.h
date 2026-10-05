// Minimal Arduino-ESP32 stub - ONLY so `make check-esp32` can syntax-check hal_esp32.cpp/main.cpp on a PC.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define CHANGE 4
#define ADC_11db 3
#define SERIAL_8N1 0x800001c
void pinMode(int, int); void digitalWrite(int, int); int digitalRead(int);
int digitalPinToInterrupt(int); void attachInterrupt(int, void (*)(), int);
void noInterrupts(); void interrupts();
uint32_t millis(); uint32_t micros(); void delay(uint32_t); void yield();
void ledcSetup(int, int, int); void ledcAttachPin(int, int); void ledcWrite(int, uint32_t);
void analogReadResolution(int); void analogSetPinAttenuation(int, int); int analogRead(int);
class HardwareSerial {
 public:
  void begin(unsigned long, uint32_t = 0, int = -1, int = -1); void setRxBufferSize(size_t);
  int available(); int read(); size_t write(const uint8_t *, size_t);
};
extern HardwareSerial Serial, Serial2;
