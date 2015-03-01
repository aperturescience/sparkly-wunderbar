/* Library to connect Spark Core to the Wunderbar Bridge Module.
** Author: Seppe Stas
** Based on WunderBarBridge for Arduino by Daniel Mancuso & Khaled Osman
** Relayr.io
*/

#include "application.h"

#ifndef _GROVE_BRIDGE_H_
#define _GROVE_BRIDGE_H_

#define PAYLOAD_MAX_LENGTH  19	//Max Payload Length for the Bridge UART packet.

typedef enum
{
    BRIDGE_COMM_WRITE_UP_CHANNEL  = 0x01,
    BRIDGE_COMM_READ_UP_CHANNEL   = 0x02,
    BRIDGE_COMM_READ_DOWN_CHANNEL = 0x03,
    BRIDGE_COMM_ACK               = 0x04,
    BRIDGE_COMM_NACK              = 0x05,
    BRIDGE_COMM_PING              = 0x06,
    BRIDGE_COMM_RCV_FROM_BLE      = 0x07,
    BRIDGE_COMM_NCONN             = 0x08,
}
bridge_commands_t;

typedef struct
{
  uint8_t command;
  uint8_t length;
  uint8_t payload[PAYLOAD_MAX_LENGTH];
  uint16_t  crc;
} bridge_comm_t;

typedef struct
{
  uint8_t length;
  uint8_t payload[PAYLOAD_MAX_LENGTH];
} bridge_payload_t;

class Bridge
{
	public:
    //constructors
    Bridge(void); //No Debug output
	Bridge(int32_t baudrate); //use debug on hardware Serial0
		
    //public methods:
    bool begin();
    bool sendData(uint8_t payload[], int size);
    void processSerial(void);
    bool checkConnection(void);
    bridge_payload_t getData(void);

    //variables:
    bool newData;
    bool useDebugOutput;

	private:
    //methods:
		uint16_t crc16Compute(uint8_t * p_data, int size, uint8_t * p_crc);
		void dumpPacket(bridge_comm_t packet);
		bridge_comm_t createUpPacket(uint8_t * payload, int length, uint8_t * outBuffer);

    //variables:
    bool commandReceived;
    int32_t   _baudrate;
    uint8_t   _rx_pin;
    uint8_t   _tx_pin;
    bool _bridgeConnected;
    uint8_t outputBuffer[PAYLOAD_MAX_LENGTH];

    struct{
      bridge_comm_t channel;
      bool packet_ok;
      uint8_t rec_bytes;
      uint8_t payload_c;
    } down;
};

#endif