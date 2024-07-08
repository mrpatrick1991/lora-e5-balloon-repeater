#include <Arduino.h>
#include <RadioLib.h>
#include <stm32wlxx_hal.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
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
enum STATE {
  LISTEN,         // receive mode, wait for a packet to arrive
  START_TRANSMIT, // re-transmit the packet
  WAIT_TRANSMIT,  // await packet transmission completion or timeout
  END_TRANSMIT    // finish transmitting the packet and clean-up, then go back to receive mode.
};

// GPS 
TinyGPSPlus gps;
HardwareSerial gps_serial(GPS_RX,GPS_TX);

// globals
int radio_state = 0;
byte rx_buffer[RX_BUFFER_MAX_SIZE];
int rx_buffer_size = 0;
long packet_tx_ms_clock = 0;
long debug_print_ms_clock = 0;
enum STATE state;
volatile bool packet_flag = false;

// callback functions for when packets are received or transmitted
void set_packet_flag(void) {
    packet_flag = true;
}

bool check_radio_state(int state, bool reset_on_error) {
  if (state != RADIOLIB_ERR_NONE) {
    D_print(F("radio failure, code: "));
    D_println(state);
    if (reset_on_error) {
      delay(1000); //if the radio failed to initialize, there is a hardware problem. wait a moment and attempt a system reset.
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

  D_println(F("*** start ***"));
  D_print(F("[GPS]: initializing... "));
  gps_serial.begin(GPS_BAUD);
  if (GPS_USE_ENABLE_PIN) {
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    digitalWrite(GPS_ENABLE_PIN, LOW);
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

  radio_state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SPREAD_FACTOR, LORA_CR, LORA_SYNCWORD, LORA_TXPOWER_DBM, LORA_PREAMBLE_LEN);
  check_radio_state(radio_state, true);

  D_print(F("[STM32WL]: setting TCXO voltage... "));
  radio_state = radio.setTCXO(TCXO_VOLT);
  check_radio_state(radio_state, true);

  radio.setDio1Action(set_packet_flag);

  D_print(F("[STM32WL]: starting recieve... "));
  radio_state = radio.startReceive();
  check_radio_state(radio_state, true);

  state = LISTEN; // end of setup, start listening for incoming packets
  D_println(F("startup finished."));
}

void loop() {

  while (gps_serial.available()) {
    gps.encode(gps_serial.read()); // read NMEA characters from the GPS as they come in
  }

  if (DEBUG) { // print GPS data every 5 seconds if debug is enabled
    if (millis() - debug_print_ms_clock > 5000l) {
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

  switch (state) {

    case LISTEN:
      if (packet_flag) {
        packet_flag = false;
        rx_buffer_size = radio.getPacketLength();
        radio_state = radio.readData(rx_buffer, rx_buffer_size);
  
        if (radio_state == RADIOLIB_ERR_NONE) {
          D_print(F("[STM32WL]: received packet: ")); // if debug enabled, print the packet contents and snr/rssi
          for (int i=0; i< rx_buffer_size; i++) {
            D_print(rx_buffer[i], HEX);
          }
          D_println();
          D_print(F("[STM32WL]: RSSI: "));
          D_print(radio.getRSSI());
          D_println(F(" dBm"));

          D_print(F("[STM32WL]: SNR: "));
          D_print(radio.getSNR());
          D_println(F(" dB"));

          state = START_TRANSMIT; // a valid packet was received, so start repeating it.
          D_print(F("[STATE MACHINE]: transition to state: "));
          D_println(state);
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
      state = WAIT_TRANSMIT;
      D_print(F("[STATE MACHINE]:  transition to state: "));
      D_println(state);
      break;
      
    case WAIT_TRANSMIT: // we are waiting for packet transmission to finish. we cannot receive during this time.
      if (packet_flag) {
        packet_flag = false;
        D_print(F("[STM32WL]: transmission finished, time elapsed: "));
        D_print(millis() - packet_tx_ms_clock);
        D_println(F(" ms."));

        state = END_TRANSMIT;
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(state);
        break;
      }
      else if (millis() - packet_tx_ms_clock > PACKET_TX_TIMEOUT_SEC*1000l) {
        D_println(F("[STM32WL]: transmission timed out "));  // timed out waiting for the transmission to finish.
        state = END_TRANSMIT;
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(state);
        break;     
      }
      else {
        break; // still waiting, do nothing.
      }

    case END_TRANSMIT:
      D_print(F("[STM32WL]: finishing transmit "));
      radio_state = radio.finishTransmit();
      check_radio_state(radio_state, false);

      D_print(F("[STM32WL]: starting receive "));
      radio_state = radio.startReceive();
      // if we fail to put the radio back into receive mode, we could get stuck here, so reset the mcu. 
      check_radio_state(radio_state, true); 
      
      state = LISTEN; // go back to listening
      D_print(F("[STATE MACHINE]: transition to state: "));
      D_println(state);
      break;
  }
}