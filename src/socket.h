// Socket Library
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

#ifndef SOCKET_H_
#define SOCKET_H_

#include <stdint.h>
#include <stdbool.h>
#include "ip.h"

// UDP/TCP socket (holds information about the UDP/TCP conversation)
// Internal database kept, so this will always be the information about the server (NOT THE REDBOARD)
typedef struct _socket
{
    uint8_t remoteIpAddress[4];
    uint8_t remoteHwAddress[6];
    uint16_t remotePort;
    uint16_t localPort;
    uint32_t sequenceNumber;
    uint32_t acknowledgementNumber;
    uint8_t  state;
} socket;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSockets();
socket * newSocket();
socket * getSocket(uint8_t instance);
void deleteSocket(socket *s);
void getSocketInfoFromArpResponse(etherHeader *ether, socket *s);
void getSocketInfoFromUdpPacket(etherHeader *ether, socket *s);
void getSocketInfoFromTcpPacket(etherHeader *ether, socket *s);

#endif

