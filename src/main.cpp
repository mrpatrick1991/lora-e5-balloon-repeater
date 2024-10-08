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
enum STATE {
  SEND_TELEMETRY, // check if it is time to send meshtastic telemetry, then do so.
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
uint8_t packet_buffer[PACKET_MAX_SIZE]; // buffer to send over the air as a radio packet
int packet_buffer_size = 0;

pb_byte_t data_buffer[237]; // container for the Data protobuf encoded bytes
int data_buffer_size = 0;

long packet_tx_ms_clock = 0;
long debug_print_ms_clock = 0;
long nodeinfo_ms_clock = 0;
long position_ms_clock = 0;
enum STATE state;
volatile bool packet_flag = false;

// protobufs
uint8_t pb_buffer[220];   // hold encoded protobuf bytes
size_t pb_message_length; // track protobuf buffer size
int pb_buffer_size = 0;

meshtastic_Data pb_data = meshtastic_Data_init_default;          // wrapper protobuf around position or node info
meshtastic_Position pb_pos = meshtastic_Position_init_zero;      // position packet protobuf (latitude, longitude, etc)
meshtastic_User pb_nodeinfo = meshtastic_User_init_zero;         // node info packet protobuf (name, device type, etc)

// callback functions for when packets are received or transmitted
void set_packet_flag(void) {
    packet_flag = true;
}

bool check_radio_state(int radio_state, bool reset_on_error) {
  if (radio_state != RADIOLIB_ERR_NONE) {
    D_print(F("radio failure, code: "));
    D_println(radio_state);
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

  memset(packet_buffer,0,PACKET_MAX_SIZE); // clear the receive buffer
  packet_buffer_size = 0;

  D_print(F("[GPS]: initializing... "));
  gps_serial.begin(GPS_BAUD);

  debug_print_ms_clock = millis();
  bool gps_ok = false; 

  while(millis() - debug_print_ms_clock <= 5000l) { // give the GPS 5 seconds to provide valid NMEA data
    while (gps_serial.available()) {
      gps.encode(gps_serial.read()); 
    }
    if (gps.charsProcessed() > 0 && gps.passedChecksum() > 0) { // we received data and at least one valid sentence
      D_println("OK");
      break;
    }
  }

  if (!gps_ok) {
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

  state = SEND_TELEMETRY; // end of setup, enter state machine.
  D_println(F("[STM32WL]: startup finished, entering state machine."));  
}

void loop() {

  while (gps_serial.available()) { // read NMEA characters from the GPS as they come in
    gps.encode(gps_serial.read()); 
  }

  if (DEBUG) { // periodically print GPS data if debug is enabled
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

  switch (state) {

    case SEND_TELEMETRY:

      // check if enough time has elapsed to send node info or position. if it has, send. otherwise, go to 
      // the receive state. 
      if (millis() - nodeinfo_ms_clock > NODEINFO_SEND_SEC*1000l) {
        nodeinfo_ms_clock = millis();
        // fill the packet buffer with node info here

        D_println("[PROTOBUF]: assembling node info protobuf.");
        strcpy(pb_nodeinfo.id, NODE_ID);
        strcpy(pb_nodeinfo.long_name, NODE_LONG_NAME);
        strcpy(pb_nodeinfo.short_name, NODE_SHORT_NAME);
        pb_nodeinfo.hw_model = meshtastic_HardwareModel_UNSET;
        pb_nodeinfo.is_licensed = false;
        pb_nodeinfo.role = meshtastic_Config_DeviceConfig_Role_ROUTER;

        memset(data_buffer, 0, sizeof(data_buffer)); 
        data_buffer_size = 0;
        pb_ostream_t stream = pb_ostream_from_buffer(data_buffer, sizeof(data_buffer));
        pb_encode(&stream, meshtastic_User_fields, &pb_nodeinfo);
        data_buffer_size = stream.bytes_written;
        stream.bytes_written = 0;

        pb_data.portnum = meshtastic_PortNum_NODEINFO_APP;
        memcpy(pb_data.payload.bytes, data_buffer, sizeof(data_buffer));
        pb_data.payload.size = data_buffer_size;
        pb_data.want_response = false;

        memset(pb_buffer, 0, sizeof(pb_buffer)); 
        pb_buffer_size = 0;
        pb_ostream_t stream2 = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
        int pb_status = pb_encode(&stream2, meshtastic_Data_fields, &pb_data);
        pb_buffer_size = stream2.bytes_written;
        stream2.bytes_written = 0;

        /*byte packet_header[16];
        memset(packet_header,0,sizeof(packet_header));
        uint8_t dst[] = {0xFF,0xFF,0xFF,0xFF};
        memcpy(packet_header,dst,sizeof(dst)); // dest address broadcast

        uint8_t dest_address[] = {0xA0,0xA0,0xA0,0xA0};
        memcpy(packet_header+4,dest_address,sizeof(dest_address));

        uint8_t nodeid[] = {0xFF,0xFF,0xFF,0xFF};
        memcpy(packet_header+8,nodeid,sizeof(nodeid)); 

        uint8_t packetid[] = {0xA1,0xB1,0xC1,0xD1};
        memcpy(packet_header+12,packetid,sizeof(packetid)); 

        uint8_t flags[] = {0xD2};
        memcpy(packet_header+13,flags,sizeof(flags));

        uint8_t hash[] = {0xD3};
        memcpy(packet_header+14,hash,sizeof(hash));

        memcpy(packet_buffer,packet_header,sizeof(packet_header));*/
        memcpy(packet_buffer+16,pb_buffer,sizeof(pb_buffer));



        if (!pb_status) {
          D_print("[PROTOBUF]: encoding failed, error: ");
          D_println(PB_GET_ERROR(&stream));
        }
        D_print("[PROTOBUF]: packet buffer contents are: ");
        if (DEBUG) {
            for (int i=0; i< packet_buffer_size; i++) {
              D_print(packet_buffer[i], HEX);
            }
            D_println();
          }
        D_print("[PROTOBUF]: packet buffer size is: ");
        D_println(packet_buffer_size);

        state = START_TRANSMIT;
        D_println(F("[STATE MACHINE]: node info needs to be sent."));
        D_print(F("[STATE MACHINE]: transition to state: "));
        D_println(state);
        break;
      }

      /*if (millis() - position_ms_clock > NODEINFO_SEND_SEC*1000l) {
        position_ms_clock = millis();
        // fill the packet buffer with position here
        state = START_TRANSMIT;
        D_print(F("[STATE MACHINE]: position needs to be sent. transition to state: "));
        D_println(state);
        break;
      }*/

      else {
        state = LISTEN;
        D_print(F("[STATE MACHINE]: no telemetry needs to be sent. transition to state: "));
        D_println(state);
        break;
      }


    case LISTEN:
      
      // keep track of how long is spent in this state. if it exceeds the telemetry transmit interval, send telemetry.
      // otherwise, stay in receive mode waiting for a packet.
      if (millis() - nodeinfo_ms_clock > NODEINFO_SEND_SEC*1000l || millis() - position_ms_clock > POSITION_SEND_SEC*1000l) {
        state = SEND_TELEMETRY;
        D_print(F("[STATE MACHINE]: check for telemetry to send. transition to state: "));
        D_println(state);
        break;
      }

      if (packet_flag) {
        packet_flag = false;
        packet_buffer_size = radio.getPacketLength();
        radiolib_state = radio.readData(packet_buffer, packet_buffer_size);
  
        if (radiolib_state == RADIOLIB_ERR_NONE) {
          D_print(F("[STM32WL]: received packet: ")); // if debug enabled, print the packet contents and snr/rssi
          if (DEBUG) {
            for (int i=0; i< packet_buffer_size; i++) {
              D_print(packet_buffer[i], HEX);
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
          D_print(packet_buffer_size);
          D_println(F(""));

          state = START_TRANSMIT; // a valid packet was received, so start repeating it.
          D_print(F("[STATE MACHINE]: transition to state: "));
          D_println(state);
          break;
        }

        else { // there was a problem reading the packet, drop it and then keep receiving.
          memset(packet_buffer, 0, PACKET_MAX_SIZE);
          packet_buffer_size = 0;
          D_print(F("[STM32WL]: receive failed, code: ")); 
          D_println(radiolib_state); 
          break;
        }
      }
    break;

    case START_TRANSMIT:
      packet_flag = false; // ISR sets to true when the transmission finishes or a packet is received
      radio.startTransmit(packet_buffer,packet_buffer_size); // start transmitting from the packet buffer
      D_println("[STM32WL]: transmit started.");
      packet_tx_ms_clock = millis(); // keep track of how long the packet takes to transmit.
      state = WAIT_TRANSMIT;
      D_print(F("[STATE MACHINE]: transition to state: "));
      D_println(state);
      break;
      
    case WAIT_TRANSMIT: // we are waiting for packet transmission to finish. and cannot receive during this time.
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
      radiolib_state = radio.finishTransmit();
      check_radio_state(radiolib_state, false);

      D_print(F("[STM32WL]: starting receive "));
      memset(packet_buffer,0,PACKET_MAX_SIZE); // clear the receive buffer
      packet_buffer_size = 0;

      radiolib_state = radio.startReceive();
      // if we fail to put the radio back into receive mode, we could get stuck here, so reset the mcu. 
      check_radio_state(radiolib_state, true); 

      state = SEND_TELEMETRY; // go back to the first state
      D_print(F("[STATE MACHINE]: transition to state: "));
      D_println(state);
      break;
  }
}