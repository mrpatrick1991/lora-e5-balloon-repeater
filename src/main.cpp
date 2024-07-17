#include <Arduino.h>
#include <RadioLib.h>
#include <stm32wlxx_hal.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "meshtastic/mesh.pb.h"
#include "config.h"

// hardware definitions, specific to the LoRa e5 from Seeed Studio
STM32WLx radio = new STM32WLx_Module();

// hardware pin assignments specific to the LoRa e5 module
static const uint32_t rfswitch_pins[] = {PA4,  PA5, PA5, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE, {LOW, LOW, LOW}},
  {STM32WLx::MODE_RX, {HIGH, LOW, LOW}},
  {STM32WLx::MODE_TX_LP, {LOW, HIGH, HIGH}}, // No LP mode on the LoRa e5, just HP
  {STM32WLx::MODE_TX_HP, {LOW, HIGH, HIGH}},
  END_OF_MODE_TABLE,
};

// state machine 
enum RADIO_STATE {
  LISTEN,         // receive mode, wait for a packet to arrive
  START_TRANSMIT, // re-transmit the packet
  WAIT_TRANSMIT,  // await packet transmission completion or timeout
  END_TRANSMIT    // finish transmitting the packet and clean-up, then go back to receive mode.
};

// GPS 
TinyGPSPlus gps;
HardwareSerial gps_serial(GPS_RX,GPS_TX);

// globals
int radiolib_state = 0;
byte rx_buffer[RX_BUFFER_MAX_SIZE];
int rx_buffer_size = 0;
long packet_tx_ms_clock = 0;
long debug_print_ms_clock = 0;
long nodeinfo_ms_clock = 0;
long position_ms_clock = 0;
enum RADIO_STATE radio_state;
volatile bool packet_flag = false;

// protobufs
uint8_t pb_buffer[512];   // hold encoded protobuf bytes
size_t pb_message_length; // track protobuf buffer size
meshtastic_Position pb_pos = meshtastic_Position_init_zero;      // position packet protobuf (latitude, longitude, etc)
meshtastic_NodeInfo pb_nodeinfo = meshtastic_NodeInfo_init_zero; // node info packet protobuf (name, device type, etc)
pb_ostream_t stream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));

// callback functions for when packets are received or transmitted
void set_packet_flag(void) {
    packet_flag = true;
}

bool check_radio_state(int state, bool reset_on_error) {
  if (state != RADIOLIB_ERR_NONE) {
    D_print(F("radio failure, code: "));
    D_println(state);
    if (reset_on_error) {
      delay(1000); // if the radio failed to initialize, there is a hardware problem. wait a moment then attempt a reset.
      NVIC_SystemReset();
    }
    return false;
  }
  else {
    D_println(F("OK"));
    return true;
  }
}

void setup() {

  // hardware
  D_SerialBegin(DEBUG_SERIAL_BAUD);

  D_println(F("[MCU]: start"));

  memset(rx_buffer,0,RX_BUFFER_MAX_SIZE); // clear the receive buffer
  rx_buffer_size = 0;

  D_print(F("[GPS]: initializing... "));
  gps_serial.begin(GPS_BAUD);
  if (GPS_USE_ENABLE_PIN) {
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    digitalWrite(GPS_ENABLE_PIN, HIGH);
  }

  debug_print_ms_clock = millis();
  bool gps_ok = false; 
  while(millis() - debug_print_ms_clock <= 5000l) { // give the GPS 5 seconds to provide valid NMEA data
    while (gps_serial.available()) {
      gps.encode(gps_serial.read()); 
    }
    if (gps.charsProcessed() > 0 && gps.passedChecksum() > 0) {
      gps_ok = true;
      D_println("OK");
      break;
    }
  }

  if (!gps_ok) {
    delay(1000); // wait a second
    D_println(F("[GPS]: not receiving valid NMEA sentences. System reset."));
    NVIC_SystemReset(); // if no valid data from the GPS is received, it's a hardware problem, try a system reset.
  }

  D_print(F("[STM32WL]: initializing... "));
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  radiolib_state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SPREAD_FACTOR, LORA_CR, LORA_SYNCWORD, LORA_TXPOWER_DBM, LORA_PREAMBLE_LEN);
  check_radio_state(radiolib_state, true);

  D_print(F("[STM32WL]: setting TCXO voltage... "));
  radiolib_state = radio.setTCXO(TCXO_VOLT);
  check_radio_state(radiolib_state, true);

  radio.setDio1Action(set_packet_flag);

  D_print(F("[STM32WL]: starting recieve... "));
  radiolib_state = radio.startReceive();
  check_radio_state(radiolib_state, true);

  radio_state = LISTEN; // end of setup, start listening for incoming packets
  D_println(F("[MCU]: startup finished, running state machine."));  

}

void loop() {

  while (gps_serial.available()) {
    gps.encode(gps_serial.read()); // read NMEA characters from the GPS as they come in
  }

  if (DEBUG) { // print GPS data if debug is enabled
    if (millis() - debug_print_ms_clock > GPS_DEBUG_PRINT_SEC*1000l) {
      debug_print_ms_clock = millis();
      D_print(F("[GPS]: characters read: "));
      D_println(gps.charsProcessed());
      D_print(F("[GPS]: good checksums: "));
      D_println(gps.passedChecksum());
      D_print(F("[GPS]: satellites: "));
      D_println(gps.satellites.value());
      D_print(F("[GPS]: latitude (deg): "));
      D_println(gps.location.lat(),6);
      D_print(F("[GPS]: longitude (deg): "));
      D_println(gps.location.lng(),6);
      D_print(F("[GPS]: altitude (m): "));
      D_println(gps.altitude.meters());
    }
  }

  switch (radio_state) {

    case LISTEN:

      if (millis() - nodeinfo_ms_clock > NODEINFO_SEND_SEC*1000l) {
        node_info_ms_clock = millis(); 
        D_println(F("[STATE MACHINE]: send NodeInfo packet"));
        memset(rx_buffer,0,RX_BUFFER_MAX_SIZE); 
        rx_buffer_size = 0;

        // assemble the packet here
        radio_state = START_TRANSMIT; // a valid packet was received, so start repeating it.
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(radio_state);
        break;
      }

      if (packet_flag) {
        packet_flag = false;
        rx_buffer_size = radio.getPacketLength();
        radiolib_state = radio.readData(rx_buffer, rx_buffer_size);
  
        if (radiolib_state == RADIOLIB_ERR_NONE) {
          D_print(F("[STM32WL]: received packet: ")); // if debug enabled, print the packet contents and snr/rssi
          if (DEBUG) {
            for (int i=0; i< rx_buffer_size; i++) {
              D_print(rx_buffer[i], HEX);
            }
          }
          D_println();
          D_print(F("[STM32WL]: RSSI: "));
          D_print(radio.getRSSI());
          D_println(F(" dBm"));

          D_print(F("[STM32WL]: SNR: "));
          D_print(radio.getSNR());
          D_println(F(" dB"));

          D_print(F("[STM32WL]: bytes: "));
          D_print(rx_buffer_size);
          D_println(F(""));

          radio_state = START_TRANSMIT; // a valid packet was received, so start repeating it.
          D_print(F("[STATE MACHINE]: transition to state: "));
          D_println(radio_state);
          break;
        }

        else { // there was a problem reading the packet, drop it and then keep receiving.
          memset(rx_buffer, 0, RX_BUFFER_MAX_SIZE);
          rx_buffer_size = 0;
          D_print(F("[STM32WL]: receive failed, code: ")); 
          D_println(radio_state); 
          break;
        }
      }
    break;

    case START_TRANSMIT:
      packet_flag = false; // ISR sets to true when the transmission finishes or a packet is received
      radio.startTransmit(rx_buffer,rx_buffer_size); // start transmitting from the packet buffer
      packet_tx_ms_clock = millis(); // keep track of how long the packet takes to transmit.
      radio_state = WAIT_TRANSMIT;
      D_print(F("[STATE MACHINE]:  transition to state: "));
      D_println(radio_state);
      break;
      
    case WAIT_TRANSMIT: // we are waiting for packet transmission to finish. and cannot receive during this time.
      if (packet_flag) {
        packet_flag = false;
        D_print(F("[STM32WL]: transmission finished, time elapsed: "));
        D_print(millis() - packet_tx_ms_clock);
        D_println(F(" ms."));

        radio_state = END_TRANSMIT;
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(radio_state);
        break;
      }
      else if (millis() - packet_tx_ms_clock > PACKET_TX_TIMEOUT_SEC*1000l) {
        D_println(F("[STM32WL]: transmission timed out "));  // timed out waiting for the transmission to finish.
        radio_state = END_TRANSMIT;
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(radio_state);
        break;     
      }
      else {
        break; // still waiting, do nothing.
      }

    case END_TRANSMIT:
      D_print(F("[STM32WL]: finishing transmit "));
      radiolib_state = radio.finishTransmit();
      check_radio_state(radiolib_state, false);

      D_print(F("[STM32WL]: starting receive "));
      memset(rx_buffer,0,RX_BUFFER_MAX_SIZE); // clear the receive buffer
      rx_buffer_size = 0;

      radiolib_state = radio.startReceive();
      // if we fail to put the radio back into receive mode, we could get stuck here, so reset the mcu. 
      check_radio_state(radiolib_state, true); 

      radio_state = LISTEN; // go back to listening
      D_print(F("[STATE MACHINE]: transition to state: "));
      D_println(radio_state);
      break;
  }
}