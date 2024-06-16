#include <Arduino.h>
#include <RadioLib.h>
#include "stm32wlxx_hal.h"

// config
#define DEBUG_SERIAL_BAUD 115200
#define GPS_SERIAL_BAUD 9600
#define DEBUG_SERIAL Serial

// debug 
#define DEBUG true

#if DEBUG
  #define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
  #define D_print(...)    Serial.print(__VA_ARGS__)
  #define D_write(...)    Serial.write(__VA_ARGS__)
  #define D_println(...)  Serial.println(__VA_ARGS__)
#else
  #define D_SerialBegin(...)
  #define D_print(...)
  #define D_write(...)
  #define D_println(...)
#endif

// hardware definitions
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] =
                         {PA4,  PA5, PA5, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE, {LOW, LOW, LOW}},
  {STM32WLx::MODE_RX, {HIGH, LOW, LOW}},
  {STM32WLx::MODE_TX_LP, {LOW, HIGH, HIGH}}, // No LP mode on the LoRa e5, just HP
  {STM32WLx::MODE_TX_HP, {LOW, HIGH, HIGH}},
  END_OF_MODE_TABLE,
};

// globals
int radio_state = 0;
volatile bool receivedFlag = false;    
volatile bool transmittedFlag = false;     
int rxBufferSize = 0;
byte rxBuffer[256];

// callback function when packets are received or transmitted
void setPacketFlag(void) {
  uint16_t irqstatus = radio.getIrqStatus();
  if (irqstatus == RADIOLIB_SX126X_IRQ_RX_DONE) {
    receivedFlag = true;
  }
  else if (irqstatus == RADIOLIB_SX126X_IRQ_TX_DONE) {
    transmittedFlag = true;
  }
  else {
    receivedFlag = false;
    transmittedFlag = false;
  }
}

void setup() {

  // hardware init
  D_SerialBegin(DEBUG_SERIAL_BAUD);

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  D_println(F("** startup **"));
  D_print(F("[STM32WL] initializing... "));
  radio_state = radio.begin(906.875, 250.0, 11, 5, 0x4B, 22, 16);

  if (radio_state == RADIOLIB_ERR_NONE) {
    D_println(F("OK"));
  }
  else {
    D_print(F("failed, code: "));
    D_println(radio_state);
    while (1) {};
  }

  D_print(F("[STM32WL] setting TCXO voltage... "));
  radio_state = radio.setTCXO(1.7);

  if (radio_state == RADIOLIB_ERR_NONE) {
    D_println(F("OK"));
  }
  else {
    D_print(F("failed, code: "));
    D_println(radio_state);
    while (1) {};
  }

  radio.setDio1Action(setPacketFlag);

  D_print(F("[STM32WL] starting recieve... "));
  radio_state = radio.startReceive();

  if (radio_state == RADIOLIB_ERR_NONE) {
    D_println(F("OK"));
  }
  else {
    D_print(F("failed, code: "));
    D_println(radio_state);
    while (1) {};
  }
}

long last_ms = 0;

void loop() {

  if (receivedFlag) {
    rxBufferSize = radio.getPacketLength();
    radio_state = radio.readData(rxBuffer,rxBufferSize);
  
    if (radio_state == RADIOLIB_ERR_NONE) {
      D_print(F("[STM32WL] received packet: "));
      for (int i=0; i< rxBufferSize; i++) {
        D_print(rxBuffer[i], HEX);
      }
      D_println();

      D_print(F("RSSI:\t\t"));
      D_print(radio.getRSSI());
      D_println(F(" dBm"));

      D_print(F("SNR:\t\t"));
      D_print(radio.getSNR());
      D_println(F(" dB"));
    }
    else if (radio_state == RADIOLIB_ERR_CRC_MISMATCH) {
      D_println(F("CRC error")); // packet was received, but is malformed
    }
    else {
      D_print(F("failed, code: "));
      D_println(radio_state);
    }
    
    D_print("[STM32WL] starting receive... ");
    radio_state = radio.startReceive();
    if (radio_state == RADIOLIB_ERR_NONE) {
      D_println(F("OK"));
    }
    else {
      D_print(F("failed, code: "));
      D_println(radio_state);
    }

    receivedFlag = false;
    rxBufferSize = 0;
    memset(rxBuffer,0,sizeof(rxBuffer));

  }

  if (transmittedFlag) {
    transmittedFlag = false;
    radio.finishTransmit();
  }

  if (millis() - last_ms > 5000) {
    last_ms = millis();
    D_print(F("runtime: "));
    D_println(millis());
    //NVIC_SystemReset();
    //radio_state = radio.begin(906.875, 250.0, 11, 5, 0x4B, 22, 16);
    //radio_state = radio.startReceive();
  }
}
