#pragma once
#include <stdint.h>
typedef enum { RF24_PA_MAX = 3 } rf24_pa_dbm_e;
typedef enum { RF24_250KBPS = 2 } rf24_datarate_e;
typedef enum { RF24_CRC_16 = 2 } rf24_crclength_e;
class RF24 {
 public:
  RF24(uint16_t, uint16_t);
  bool begin(); void setChannel(uint8_t); void setDataRate(rf24_datarate_e); void setPALevel(uint8_t);
  void setCRCLength(rf24_crclength_e); void setAutoAck(bool); void enableDynamicPayloads();
  void setRetries(uint8_t, uint8_t); void openReadingPipe(uint8_t, const uint8_t *); void openWritingPipe(const uint8_t *);
  void startListening(); void stopListening(); bool write(const void *, uint8_t); bool available();
  uint8_t getDynamicPayloadSize(); void read(void *, uint8_t); void flush_rx(); uint8_t getARC();
};
