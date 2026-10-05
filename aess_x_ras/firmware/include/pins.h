// pins.h - ESP32 DevKit (30-pin, ESP32-WROOM-32) wiring. One map for every robot; the Hub only uses the radio pins.
// GPIO 34-39 are input-only (no pull-ups). GPIO 0/2/5/12/15 are boot-strapping pins, so they only drive
// things that are harmless at power-up. UART0 (USB) is the host link, so NO debug prints on Serial.
#ifndef LM_PINS_H
#define LM_PINS_H

// ---- TB6612FNG dual motor driver (STBY tied to 3V3; PWM 20 kHz)
#define PIN_MOT_L_PWM   25
#define PIN_MOT_L_IN1   26
#define PIN_MOT_L_IN2   27
#define PIN_MOT_R_PWM   14
#define PIN_MOT_R_IN1   13
#define PIN_MOT_R_IN2   33
#define MOTOR_L_INVERT  0       // flip at bring-up if "forward" spins a wheel backwards
#define MOTOR_R_INVERT  0

// ---- wheel encoders (quadrature Hall encoders on the gear motors, input-only pins -> need 3V3 push-pull or external pull-ups)
#define PIN_ENC_L_A     34
#define PIN_ENC_L_B     35
#define PIN_ENC_R_A     36
#define PIN_ENC_R_B     39
#define ENC_L_SIGN      (+1)    // the right motor is mirrored, so its sign is usually -1; verify at bring-up
#define ENC_R_SIGN      (-1)

// ---- MPU-6050 IMU (I2C, 400 kHz) - only gyro Z is used
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define MPU_ADDR        0x68

// ---- LD06 LiDAR on UART2 (230400 8N1). The LD06 only transmits, so only RX is wired.
#define PIN_LIDAR_RX    16
#define LIDAR_BAUD      230400

// ---- nRF24L01+ on VSPI (3V3 only! 10 uF capacitor across VCC/GND at the module)
#define PIN_NRF_SCK     18
#define PIN_NRF_MISO    19
#define PIN_NRF_MOSI    23
#define PIN_NRF_CSN     5
#define PIN_NRF_CE      4
#define RF_CHANNEL      76      // 2476 MHz, above the Wi-Fi channels most likely to be around
#define RF_ADDR_PREFIX  { 'L', 'M', 'N', 'T' }   // 5th byte: 0x10 hub, 0x01 Writer, 0x02 Executor

// ---- event sensors (Writer only)
#define PIN_MQ2_AO      32      // ADC1_CH4, through a 10k/15k divider (5 V -> 3.0 V). Do not use ADC2 pins with radios on.
#define MQ2_DIVIDER     0.6f    // Vadc = Vao * 0.6
#define PIN_TILT_DO     17      // SW-520D module (LM393) digital output
#define TILT_TRIPPED_LEVEL LOW  // verify polarity on the actual module
#define PIN_PIR_OUT     15      // HC-SR501 output (3.3 V push-pull)

// ---- misc
#define PIN_LED         2
#define PIN_BATT_SENSE  (-1)    // not wired in this revision (all ADC1 pins are used); battery_mv reports 0 = unknown

#endif
