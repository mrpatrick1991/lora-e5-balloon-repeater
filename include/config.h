// debug settings
#define DEBUG 1                // enable debug serial output
#define DEBUG_SERIAL Serial
#define GPS_DEBUG_PRINT_SEC 10 // print GPS data every n seconds when debug is enabled
#define DEBUG_SERIAL_BAUD 115200

// meshtastic settings
#define ENCRYPT_AES_KEY "1PG7OiApB1nwvP+rz05pAQ==" // default encryption key in base64
#define NODEINFO_SEND_SEC 10                       // send meshtastic node info packet every 5 minutes
#define POSITION_SEND_SEC 120                      // send meshtastic position packets every 2 minutes
#define MESH_HOPS 3                                // number of hops for sent meshtastic packets
#define NODE_ID "123456"                           // node number
#define NODE_LONG_NAME "HIGHBALL-1"                // long name
#define NODE_SHORT_NAME "HBA1"                     // short name

// radio settings
#define PACKET_MAX_SIZE 256
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
#define GPS_SERIAL_BAUD 9600
#define GPS_RX PB7
#define GPS_TX PB6
#define GPS_BAUD 9600

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


