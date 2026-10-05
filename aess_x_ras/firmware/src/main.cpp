// main.cpp - one firmware image per role, chosen at build time (-DFW_ROLE_WRITER / -DFW_ROLE_EXECUTOR / -DFW_ROLE_HUB).
#include <Arduino.h>
#include "hal_esp32.h"
#include "apps.h"

#if defined(FW_ROLE_WRITER)
  #define THIS_ROLE ROLE_WRITER
#elif defined(FW_ROLE_EXECUTOR)
  #define THIS_ROLE ROLE_EXECUTOR
#elif defined(FW_ROLE_HUB)
  #define THIS_ROLE ROLE_HUB
#else
  #error "Select a role: -DFW_ROLE_WRITER, -DFW_ROLE_EXECUTOR or -DFW_ROLE_HUB (see platformio.ini)"
#endif

static Esp32Hal hal;
static AppBase *app = nullptr;

void setup() {
  hal.begin(THIS_ROLE);
  RobotParams params;                      // reference-chassis defaults (include/params.h); tune after measuring the robot
#if defined(FW_ROLE_WRITER)
  app = new WriterApp(&hal, params);
#elif defined(FW_ROLE_EXECUTOR)
  app = new ExecutorApp(&hal, params);
#else
  app = new HubApp(&hal, params);
#endif
  app->begin();
  if (!hal.radioPresent()) {               // nRF24 not answering on SPI: blink fast forever so it is obvious on the bench
    for (;;) { hal.setLed(true); delay(80); hal.setLed(false); delay(80); }
  }
}

void loop() {
  app->poll();
  yield();                                  // feed the watchdog / let the radio and USB stacks run
}
