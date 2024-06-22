#include <RadioLib.h>

STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = {PC3,  PC4,  PC5, RADIOLIB_NC, RADIOLIB_NC};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
  {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},
  {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
  END_OF_MODE_TABLE,
};

volatile bool receivedFlag = false;
volatile bool transmitFlag = false;
int rxBufferSize = 0;
byte rxBuffer[256];

void setup() {
  Serial.begin(115200);

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  radio.begin(906.875, 250.0, 7, 5, 0x4B, 2, 20);
  radio.setTCXO(1.7);
  radio.setDio1Action(setRxFlag);
  radio.startReceive();

  Serial.println("ready.");
}

void setRxFlag(void) {
  receivedFlag = true;
}

void loop() {

  if(receivedFlag) {

    rxBufferSize = radio.getPacketLength();
    radio.readData(rxBuffer, rxBufferSize);

    for (int i=0; i< rxBufferSize; i++) {
      Serial.print(rxBuffer[i], HEX);
    }
    Serial.println();

    for (int i=0;i< rxBufferSize; i++) {
      Serial.write(rxBuffer[i]);
    }
    Serial.println();

    int state = radio.transmit(rxBuffer, rxBufferSize);

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

    // print measured data rate
    Serial.print(F("[STM32WL] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }

    //radio.transmit(rxBuffer,rxBufferSize);
    receivedFlag = false;
    radio.startReceive();
  }
}