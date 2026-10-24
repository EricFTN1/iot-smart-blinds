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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "arp.h"
#include "gpio.h"
#include "tcp.h"
#include "mqtt.h"
#include "uart0.h"
#include "socket.h"
#include "timer.h"

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

#define MAX_TCP_PORTS 4
#define MOTOR1_GPIO PORTE,1
#define MOTOR2_GPIO PORTE,2

#define BLUE_LED PORTF,2
#define strSize 20

uint16_t tcpPorts[MAX_TCP_PORTS];
uint8_t tcpPortCount = 0;
uint8_t tcpState[MAX_TCP_PORTS];
extern uint8_t subMsgID, unsubMsgID;

char strTopic[MAX_PACKET_SIZE], strData[MAX_PACKET_SIZE];


// ------------------------------------------------------------------------------
//  Global State Flags
// ------------------------------------------------------------------------------

bool synAckReceived = false, finReceived = false, lastAckReceived = false;
bool applicationClosed = false, ackForFin = false, finForFin = false, messagePending = false;
bool finWaitReceived = false, closingAck = false, listenSyn = false, mqttPubResponse = false, mqttUnsubAck = false;
bool sendFinAndAck = false, connectAck = false, mqttSubAck = false, mqttPublishAck = false, mqttPingAck = false;
bool isMqttConnected = false, initialSub = true;
extern bool mqttConnect;
extern bool mqttDisconnectPending;
extern bool mqttPublishPending;
extern bool mqttSubscribePending;
extern bool mqttUnsubscribePending;
extern bool mqttPingReq;

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Init TCP
void initTCP()
{
    uint8_t i = 0;
    for(i = 0; i < MAX_TCP_PORTS; i++)
    {
        tcpState[i] = TCP_CLOSED;
        tcpPorts[i] = 0;
    }
}
//Used to send ARP request then establish TCP connection
void connectTcp()
{
    uint8_t buffer[MAX_PACKET_SIZE];
    etherHeader *data = (etherHeader*) buffer;
    uint8_t remoteIpAddress[4] = {192,168,1,244};
    uint8_t localIpAddress[4];
    getIpAddress(localIpAddress);

    sendArpRequest(data, localIpAddress, remoteIpAddress);
}
//Used to send a FIN/ACK to the server to end TCP connection
void disconnectTcp()
{
    sendFinAndAck = true;
}


// Set TCP state
void setTcpState(uint8_t instance, uint8_t state)
{
    tcpState[instance] = state;
}

// Get TCP state
uint8_t getTcpState(uint8_t instance)
{
    return tcpState[instance];
}


// Determines whether packet is TCP packet
// Must be an IP packet
bool isTcp(etherHeader* ether)
{
    ipHeader *ip  = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    uint16_t tmp16;
    uint16_t tcp_size = ntohs(ip->length) - ipHeaderLength;
    tcp_size = htons(tcp_size);
    bool okay;
    uint32_t sum = 0;
    okay = (ip->protocol == PROTOCOL_TCP);
    if(okay)
    {
        // 32-bit sum over pseudo-header
        sumIpWords(ip->sourceIp, 8, &sum);
        tmp16 = ip->protocol;
        sum += (tmp16 & 0xff) << 8;
        sumIpWords(&tcp_size, 2, &sum);
        // add tcp header and data
        sumIpWords(tcp,  ntohs(tcp_size), &sum);
        okay = (getIpChecksum(sum) == 0);
    }
    return okay;
}
//Checks tcp package to see if the SYN flag is set, returns true if so
bool isTcpSyn(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool synFlag = ((htons(tcp->offsetFields) & SYN) >> 1);

    return synFlag;
}
//Checks tcp package to see if the ACK flag is set, returns true if so
bool isTcpAck(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool ackFlag = ((htons(tcp->offsetFields) & ACK) >> 4);

    return ackFlag;
}
//Checks tcp package to see if the URG flag is set, returns true if so
bool isTcpUrg(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool urgFlag = ((htons(tcp->offsetFields) & URG) >> 5);

    return urgFlag;
}
//Checks tcp package to see if the FIN flag is set, returns true if so
bool isTcpFin(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool finFlag = ((htons(tcp->offsetFields) & FIN));

    return finFlag;
}

void mqttPublishDecode(etherHeader *ether)
{
    uint8_t buffer[MAX_PACKET_SIZE];
    etherHeader *data = (etherHeader*) buffer;
    ipHeader *ip  = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    mqttFrame *mqtt = (uint8_t*)tcp + sizeof(tcpHeader);
    socket *s = getSocket(0);

    if((mqtt->controlHeader) >> 4 == 0x3) //If control header is publish (ignore bottom 4 bits)
    {
        uint8_t n = 0, i = 0, j = 0;
        uint8_t mult = 1, digit;
        uint8_t value = 0;
        uint16_t topicLen = 0, dataLen = 0;
        do
        {
            digit = mqtt->data[n++];
            value += (digit & 127) * mult;
            mult *= 128;
        }
        while((digit * 128) != 0);
        n--;
        topicLen |= mqtt->data[n++] << 8;
        topicLen |= mqtt->data[n++];
        dataLen = value - topicLen - 2;
        for(i = 0; i < topicLen; i++)
        {
            strTopic[i] = mqtt->data[n++];
        }
        for(j = 0; j < dataLen; j++)
        {
            strData[j] = mqtt->data[n++];
        }
    }
}

//Given a global state machine flag is true, will send messages, progress the state machine then set the flag false
void sendTcpPendingMessages(etherHeader *ether)
{
    uint8_t i = 0;
    uint8_t buffer[MAX_PACKET_SIZE];
    etherHeader *data = (etherHeader*) buffer;
    extern uint8_t mqttBuffer[MAX_PACKET_SIZE];
    extern uint8_t mqttLength;
    uint8_t emptyMessage[0];
    socket *s = getSocket(0);
    if(tcpPorts[0] && (tcpState[0] == TCP_CLOSED)) //If state CLOSED, send SYN, progress to SYN_SENT state
    {
        sendTcpMessage(data, s, SYN, emptyMessage, 0);
        s->state = TCP_SYN_SENT;
        tcpState[0] = TCP_SYN_SENT;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_LISTEN)) //If state LISTEN, send SYN|ACK, progress to SYN_RECEIVED
    {
        sendTcpMessage(data, s, SYN | ACK, emptyMessage, 0);
        s->state = TCP_SYN_RECEIVED;
        tcpState[0] = TCP_SYN_RECEIVED;
    }
    else if((tcpState[0] == TCP_SYN_SENT) && synAckReceived) //If state SYN_SENT and we have received an ACK to our SYN, send ACK and progress to ESTABLISHED
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        s->state = TCP_ESTABLISHED;
        tcpState[0] = TCP_ESTABLISHED;
        synAckReceived = false;
    }
    else if((tcpState[0] == TCP_ESTABLISHED) && finReceived) //If state ESTABLISHED, and we receive a FIN, send ACK and progress to CLOSE_WAIT
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        s->state = TCP_CLOSE_WAIT;
        tcpState[0] = TCP_CLOSE_WAIT;
        finReceived = false;
    }
    else if((tcpState[0] == TCP_ESTABLISHED) && sendFinAndAck) //If state ESTABLISHED, and we want to close the connection, send FIN|ACK and progress to FIN_WAIT_1
    {
        sendTcpMessage(data, s, FIN | ACK, emptyMessage, 0);
        s->state = TCP_FIN_WAIT_1;
        tcpState[0] = TCP_FIN_WAIT_1;
        sendFinAndAck = false;
    }
    else if((tcpState[0] == TCP_ESTABLISHED) && mqttConnect) //If state ESTABLISHED, and we want to connect to MQTT, send PSH|ACK with MQTT connect message
    {
        sendTcpMessage(ether, s, PSH | ACK, mqttBuffer, mqttLength);
        startPeriodicTimer(pingMqtt, 50);
        startOneshotTimer(initialMqttSub, 1);
        isMqttConnected = true;
        mqttConnect = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttDisconnectPending) //If state ESTABLISHED, and we want to disconnect from MQTT, send PSH|ACK with MQTT disconnect message
    {
        sendTcpMessage(ether, s, PSH | ACK,  mqttBuffer, mqttLength);
        stopTimer(pingMqtt);
        isMqttConnected = false;
        mqttDisconnectPending = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttPingReq) //If state ESTABLISHED, and we want to disconnect from MQTT, send PSH|ACK with MQTT disconnect message
    {
        sendTcpMessage(ether, s, PSH | ACK,  mqttBuffer, mqttLength);
        mqttPingReq = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttPingAck) //If state ESTABLISHED, and we want to disconnect from MQTT, send PSH|ACK with MQTT disconnect message
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        restartTimer(pingMqtt);
        mqttPingAck = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttPublishPending)
    {
        sendTcpMessage(ether, s, PSH | ACK, mqttBuffer, mqttLength);
        restartTimer(pingMqtt);
        mqttPublishPending = false;
        mqttPubResponse = true;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttSubscribePending)
    {
        sendTcpMessage(ether, s, PSH | ACK, mqttBuffer, mqttLength);
        restartTimer(pingMqtt);
        mqttSubscribePending = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttUnsubscribePending)
    {
        sendTcpMessage(ether, s, PSH | ACK, mqttBuffer, mqttLength);
        restartTimer(pingMqtt);
        mqttUnsubscribePending = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttSubAck)
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        restartTimer(pingMqtt);
        mqttSubAck = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttUnsubAck)
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        restartTimer(pingMqtt);
        mqttUnsubAck = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && mqttPublishAck)
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        restartTimer(pingMqtt);
        mqttPublishAck = false;
    }
    else if ((tcpState[0] == TCP_ESTABLISHED) && connectAck) //If state is ESTABLISHED and we received an MQTT connect ACK then send back an ACK
    {
        sendTcpMessage(ether, s, ACK, emptyMessage, 0);
        connectAck = false;
    }
    else if((tcpState[0] == TCP_FIN_WAIT_1) && ackForFin) //If state is FIN_WAIT_1 and we have received an ACK progress to FIN_WAIT_2
    {
        s->state = TCP_FIN_WAIT_2;
        tcpState[0] = TCP_FIN_WAIT_2;
        ackForFin = false;
    }
    else if((tcpState[0] == TCP_FIN_WAIT_1) && finForFin) //If state is FIN_WAIT_1 and we received a FIN, send ACK back and progress to CLOSING
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        s->state = TCP_CLOSING;
        tcpState[0] = TCP_CLOSING;
        finForFin = false;
    }
    else if ((tcpState[0] == TCP_FIN_WAIT_2) && finWaitReceived) //If state is FIN_WAIT_2 and we receive a FIN, send an ACK and progress to TIME_WAIT
    {
        sendTcpMessage(data, s, ACK, emptyMessage, 0);
        s->state = TCP_TIME_WAIT;
        tcpState[0] = TCP_TIME_WAIT;
        finWaitReceived = false;
    }
    else if ((tcpState[0] == TCP_CLOSING) && closingAck) //If state is CLOSING and we receive an ACK, progress to TIME_WAIT
    {
        s->state = TCP_TIME_WAIT;
        tcpState[0] = TCP_TIME_WAIT;
        closingAck = false;
    }
    else if((tcpState[0] == TCP_CLOSE_WAIT) /*&& applicationClosed ADD LATER WHEN 0 UNDERSTAND*/)
    {
        sendTcpMessage(data, s, FIN | ACK, emptyMessage, 0);
        s->state = TCP_LAST_ACK;
        tcpState[0] = TCP_LAST_ACK;
        applicationClosed = false;
    }
    else if((tcpState[0] == TCP_LAST_ACK) && lastAckReceived) //If state is LAST_ACK and we receive an ACK then progress state to TCP_CLOSED and close socket
    {
        s->state = TCP_CLOSED;
        tcpState[0] = TCP_CLOSED;
        deleteSocket(s);
        tcpPorts[0] = 0;
        lastAckReceived = false;
    }
}

void processTcpResponse(etherHeader *ether)
{
    uint8_t i = 0;
    uint8_t buffer[MAX_PACKET_SIZE];
    etherHeader *data = (etherHeader*) buffer;
    ipHeader *ip  = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = ip->size * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    mqttFrame *mqtt = (uint8_t*)tcp + sizeof(tcpHeader);
    uint8_t emptyMessage[0], cmp0 = 1, cmp1 = 1, cmp2 = 1, cmp3 = 1;
    char topic[] = "remote";
    char stopMessage[] = "stop";
    char LeftonMessage[] = "left_on";
    char RightonMessage[] = "right_on";
    socket *s = getSocket(0);
    extern uint8_t subVarLength;

    if(tcpPorts[0] && (tcpState[0] == TCP_CLOSED) && isTcpSyn(ether)) //If we receive a SYN while in CLOSED state, set listenSyn flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        listenSyn = true;

    }
    if(tcpPorts[0] && (tcpState[0] == TCP_SYN_SENT)) //If SYN_SENT state, set synAckReceived flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        synAckReceived = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && isTcpFin(ether)) //If we receive a FIN while in ESTABLISHED state, set finReceived flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader)));
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        finReceived = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqtt->controlHeader == 0x20) //If we receive a disconnect command from MQTT & we're in ESTABLISHED set connectAck to true
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader))); //Flipping ack/sync number and adding the length
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        connectAck = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqtt->controlHeader == 0xD0) //If we receive a disconnect command from MQTT & we're in ESTABLISHED set connectAck to true
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader))); //Flipping ack/sync number and adding the length
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        mqttPingAck = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqttPubResponse)
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader)));
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        mqttPubResponse = false;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqtt->data[2] == subMsgID && (mqtt->controlHeader >> 4) == 0x9)
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader)));
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        mqttSubAck = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqtt->data[2] == unsubMsgID && (mqtt->controlHeader >> 4) == 0xB)
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader)));
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        mqttUnsubAck = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_ESTABLISHED) && mqtt->controlHeader == 0x30)
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber) + (ntohs(ip->length) - (sizeof(tcpHeader) + sizeof(ipHeader)));
        s->sequenceNumber = ntohl(tcp->acknowledgementNumber);
        mqttPublishDecode(ether);
        char str[50];
        cmp0 = strcmp(strTopic, topic);
        cmp1 = strcmp(strData, stopMessage);
        cmp2 = strcmp(strData,  LeftonMessage);
        cmp3 = strcmp(strData,  RightonMessage);

        if(!cmp0)
        {
            if(!cmp2) //Left
            {
                setPinValue(MOTOR1_GPIO, 1);
                setPinValue(MOTOR2_GPIO, 0);
            }
            if(!cmp3) //Right
            {
                setPinValue(MOTOR1_GPIO, 0);
                setPinValue(MOTOR2_GPIO, 1);
            }
            if(!cmp1) //stop
            {
                setPinValue(MOTOR1_GPIO, 0);
                setPinValue(MOTOR2_GPIO, 0);
            }
        }
        memset(strTopic, 0, sizeof(strTopic));
        memset(strData, 0, sizeof(strData));
        mqttPublishAck = true;
    }

    if(tcpPorts[0] && (tcpState[0] == TCP_FIN_WAIT_1) && isTcpAck(ether)) //If we receive an ACK while in FIN_WAIT_1 state, set ackForFin flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        ackForFin = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_FIN_WAIT_1) && isTcpFin(ether)) //If we receive a FIN while in FIN_WAIT_1 state, set finForFin flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        finForFin = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_FIN_WAIT_2) && isTcpFin(ether)) //If we receive a FIN while in FIN_WAIT_2 state, set finWaitReceived flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        finWaitReceived = true;
    }
    if(tcpPorts[0] && (tcpState[0] == TCP_CLOSING) && isTcpAck(ether)) //If we receive an ACK while in TCP_CLOSING state, set closingAck flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        closingAck = true;

    }
    if(tcpPorts[0] && (tcpState[0] == TCP_LAST_ACK)) //If state is LAST_ACK set lastAckReceived flag and save/flip ack number
    {
        s->acknowledgementNumber = ntohl(tcp->sequenceNumber);
        lastAckReceived = true;
    }
}
//Get information from ARP response, set up TCP socket
void processTcpArpResponse(etherHeader *ether)
{
    arpPacket *arp = (arpPacket*)ether->data;
    uint8_t remoteIpAddress[4] = {192,168,1,244};
    uint8_t i = 0;
    bool isMqtt = true, found = false;
    for(i = 0; i < IP_ADD_LENGTH; i++)
    {
        if(remoteIpAddress[i] != arp->sourceIp[i])
            isMqtt = false;
    }
    if(isMqtt)
    {
        uint8_t i = 0;
        socket *s =  newSocket();
        getSocketInfoFromArpResponse(ether, s);
        // Loop through all ports, find one that is closed and open it
        for(i = 0; i < MAX_TCP_PORTS; i++)
        {
            if(tcpPorts[i] == TCP_CLOSED)
            {
                switch(i)
                {
                case 0:
                    srand(random32()); //Seed random based on clock
                    tcpPorts[i] = s->localPort = (rand() % (50000 - 45000 + 1)) + 45000; //Get a random port number between 45000 and 50000
                    s->remotePort = 1883;
                    tcpPortCount++;
                    found = true;
                    break;
                case 1:
                    srand(random32()); //Seed random based on clock
                    tcpPorts[i] = s->localPort = (rand() % (50000 - 45000 + 1)) + 45000; //Get a random port number between 45000 and 50000
                    s->remotePort = 1883;
                    tcpPortCount++;
                    found = true;
                    break;
                case 2:
                    srand(random32()); //Seed random based on clock
                    tcpPorts[i] = s->localPort = (rand() % (50000 - 45000 + 1)) + 45000; //Get a random port number between 45000 and 50000
                    s->remotePort = 1883;
                    tcpPortCount++;
                    found = true;
                    break;
                case 3:
                    srand(random32()); //Seed random based on clock
                    tcpPorts[i] = s->localPort = (rand() % (50000 - 45000 + 1)) + 45000; //Get a random port number between 45000 and 50000
                    s->remotePort = 1883;
                    tcpPortCount++;
                    found = true;
                    break;
                }
            }
            if(found)
            {
                found = false;
                break;
            }
        }
        sendTcpPendingMessages(ether);
    }
}

void setTcpPortList(uint16_t ports[], uint8_t count)
{
    uint8_t i = 0;
    if(count <= 4)
    {
        for(i = 0; i < count; i++)
        {
            tcpPorts[i] = ports[i];
        }
    }
    else
        putsUart0("Too many arguments, TCP is limited to 4 ports");
}

bool isTcpPortOpen(etherHeader *ether)
{
    uint8_t i = 0;
    ipHeader *ip = (ipHeader*)ether->data;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + (ip->size * 4));
    for(i = 0; i < MAX_TCP_PORTS; i++)
    {
        if(ntohs(tcp->destPort) == tcpPorts[i])
        {
            return true;
        }
    }

    return false;
}

//Sends/builds any TCP messages even if there is not a data field.
void sendTcpMessage(etherHeader *ether, socket *s, uint16_t flags, uint8_t data[], uint16_t dataSize)
{
    uint8_t i;
    uint8_t localHWAddress[6];
    uint8_t localIpAddress[4];
    uint8_t *copyData;
    uint16_t j;
    uint16_t tmp16;
    uint32_t sum = 0;

    getEtherMacAddress(localHWAddress);
    getIpAddress(localIpAddress);
    for(i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = s->remoteHwAddress[i];
        ether->sourceAddress[i] = localHWAddress[i];
    }
    ether->frameType = htons(TYPE_IP);

    //IP header
    ipHeader *ip = (ipHeader*)ether->data;
    ip->rev = 0x4;
    ip->size = 0x5;
    uint8_t ipHeaderLength = ip->size * 4;
    ip->typeOfService = 0;
    ip->id = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 128;
    ip->protocol = PROTOCOL_TCP;
    ip->headerChecksum = 0;
    uint16_t tcpSize = sizeof(tcpHeader) + dataSize;
    ip->length = htons(ipHeaderLength + tcpSize);
    for(i = 0; i < IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = s->remoteIpAddress[i];
        ip->sourceIp[i] = localIpAddress[i];
    }
    calcIpChecksum(ip);


    // TCP Header
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + (ip->size * 4));
    tcp->sourcePort = htons(s->localPort);
    tcp->destPort = htons(s->remotePort);
    tcp->offsetFields = htons((0x5000 | flags));
    //Sequence number and acknowledgment number change depending on the state
    switch(tcpState[0])
    {
        case(TCP_CLOSED):
            tcp->sequenceNumber = 0; //
            tcp->acknowledgementNumber = 0;  //
            s->sequenceNumber = 0;
            s->acknowledgementNumber = 0;
            break;
        case(TCP_SYN_SENT):
            tcp->sequenceNumber = htonl(1);
            s->sequenceNumber = ntohl(tcp->sequenceNumber);
            tcp->acknowledgementNumber = htonl(s->acknowledgementNumber + 1);
            break;
        case(TCP_ESTABLISHED):
            if(finReceived)
            {
                tcp->sequenceNumber = htonl(s->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber + 1);
            }
            else if(sendFinAndAck)
            {
                tcp->sequenceNumber = htonl(s->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber + 1);
            }
            else if(mqttConnect)
            {
                tcp->sequenceNumber = htonl(s->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber + 1);
            }
            else if(mqttPingReq)
            {
                tcp->sequenceNumber = htonl(s->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber);
            }
            else if(connectAck || mqttSubAck || mqttPublishAck || mqttUnsubAck || mqttDisconnectPending ||
                    mqttUnsubscribePending || mqttSubscribePending || mqttPublishPending || mqttPingAck)
            {
                tcp->sequenceNumber = htonl(s->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber);
            }
            break;
        case(TCP_FIN_WAIT_2):
                tcp->sequenceNumber = htonl(1) + tcp->sequenceNumber;
                s->sequenceNumber = ntohl(tcp->sequenceNumber);
                tcp->acknowledgementNumber = htonl(s->acknowledgementNumber + 1);
        case(TCP_CLOSE_WAIT):
            break;


    }
    tcp->windowSize = htons(1460); //Max MTU - Ether header - IP Header
    tcp->urgentPointer = 0;
    copyData = tcp->data;
    for (j = 0; j < dataSize; j++)
        copyData[j] = data[j];
    // 32-bit sum over pseudo-header
    sumIpWords(ip->sourceIp, 8, &sum);
    tmp16 = ip->protocol;
    sum += (tmp16 & 0xff) << 8;
    tcpSize = htons(tcpSize);
    sumIpWords(&tcpSize, 2, &sum);
    // add tcp header and data
    tcp->checksum = 0;
    sumIpWords(tcp,  ntohs(tcpSize), &sum);
    tcp->checksum = getIpChecksum(sum);
    putEtherPacket(ether, sizeof(etherHeader) + ipHeaderLength + (sizeof(tcpHeader) + dataSize));
}
