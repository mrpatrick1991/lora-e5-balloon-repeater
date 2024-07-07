// print debug messsages
#define DEBUG true
#define DEBUG_SERIAL Serial

// radio settings
#define DEBUG_SERIAL_BAUD 115200
#define GPS_SERIAL_BAUD 9600

#define RX_BUFFER_MAX_SIZE 256
#define LORA_FREQ_MHZ 906.875
#define LORA_BW_KHZ 250.0
#define LORA_SPREAD_FACTOR 11
#define LORA_CR 5
#define LORA_SYNCWORD 0x4B
#define LORA_TXPOWER_DBM 10
#define LORA_PREAMBLE_LEN 16
#define TCXO_VOLT 1.6
#define PACKET_TX_TIMEOUT_SEC 5 // wait at most this many seconds to finish sending a packet

// gps settings
#define GPS_RX PB7
#define GPS_TX PB6
#define GPS_BAUD 9600
#define GPS_USE_ENABLE_PIN 0
#define GPS_ENABLE_PIN PB4

#if DEBUG
  #define D_SerialBegin(...) DEBUG_SERIAL.begin(__VA_ARGS__);
  #define D_print(...) DEBUG_SERIAL.print(__VA_ARGS__)
  #define D_write(...) DEBUG_SERIAL.write(__VA_ARGS__)
  #define D_println(...) DEBUG_SERIAL.println(__VA_ARGS__)
#else
  #define D_SerialBegin(...)
  #define D_print(...)
  #define D_write(...)
  #define D_println(...)
#endif


