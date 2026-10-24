// TCP Library (includes framework only)
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: -
// Target uC:       -
// System Clock:    -

// Hardware configuration:
// -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef TCP_H_
#define TCP_H_

#include <stdint.h>
#include <stdbool.h>
#include "ip.h"
#include "socket.h"

typedef struct _tcpHeader // 20 or more bytes
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint32_t sequenceNumber;
  uint32_t acknowledgementNumber;
  uint16_t offsetFields;
  uint16_t windowSize;
  uint16_t checksum;
  uint16_t urgentPointer;
  uint8_t  data[0];
} tcpHeader;


// TCP states
#define TCP_CLOSED 0
#define TCP_LISTEN 1
#define TCP_SYN_RECEIVED 2
#define TCP_SYN_SENT 3
#define TCP_ESTABLISHED 4
#define TCP_FIN_WAIT_1 5
#define TCP_FIN_WAIT_2 6
#define TCP_CLOSING 7
#define TCP_CLOSE_WAIT 8
#define TCP_LAST_ACK 9
#define TCP_TIME_WAIT 10

// TCP offset/flags
#define FIN 0x0001
#define SYN 0x0002 // SYNC message
#define RST 0x0004
#define PSH 0x0008 // I've got data in the payload
#define ACK 0x0010 // After initial SYN, every message will have an ACK
#define URG 0x0020
#define ECE 0x0040
#define CWR 0x0080
#define NS  0x0100
#define OFS_SHIFT 12

#define defaultSocket 0
#define MAX_PACKET_SIZE 1518

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initTCP();
void processTcpResponse(etherHeader *ether);
void connectTcp();
void disconnectTcp();
void setTcpState(uint8_t instance, uint8_t state);
uint8_t getTcpState(uint8_t instance);
bool isTcp(etherHeader *ether); // Return true if the message is TCP
bool isTcpSyn(etherHeader *ether); // Use before established
bool isTcpAck(etherHeader *ether); // Use before established
void mqttPublishDecode(etherHeader *ether);
void sendTcpPendingMessages(etherHeader *ether); // Send any messages that it needs to based on the state, when you call this function
// It look at the state and dependent on the state it will send the next message (TO DO PENDING LIST THAT I NEED TO WRITE)
void processDhcpResponse(etherHeader *ether); // DO NOT NEED
void processTcpArpResponse(etherHeader *ether); // DO NOT WORRY ABOUT FOR NOW
void setTcpPortList(uint16_t ports[], uint8_t count); // Create a port list of ports that are open (allow conversation to appear on)
bool isTcpPortOpen(etherHeader *ether); // Go through the port list (stored locally)
void sendTcpMessage(etherHeader *ether, socket* s, uint16_t flags, uint8_t data[], uint16_t dataSize); // Send anything you want to anybody
//PSH turned on

#endif

