// MQTT Library (includes framework only)
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
#include "mqtt.h"
#include "socket.h"
#include "tcp.h"
#include "arp.h"
#include "timer.h"

#define MAX_PACKET_SIZE 1518

bool mqttConnect = false, mqttDisconnectPending = false, mqttPublishPending = false;
bool mqttSubscribePending = false, mqttUnsubscribePending = false, mqttPingReq = false;
// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
uint8_t mqttBuffer[MAX_PACKET_SIZE];
uint8_t mqttLength, subMsgID, subVarLength = 0, unsubMsgID;

void connectMqtt()
{
    uint8_t n = 0;

    mqttFrame *mqtt = mqttBuffer;
    mqtt->controlHeader = 0x10; //Connect type, QoS 0
    mqtt->data[n++] = 0x11;     //Remaining length
    mqtt->data[n++] = 0x0;      //Length of protocol name
    mqtt->data[n++] = 0x4;      //Length of protocol name
    mqtt->data[n++] = 0x4d;     //M
    mqtt->data[n++] = 0x51;     //Q
    mqtt->data[n++] = 0x54;     //T
    mqtt->data[n++] = 0x54;     //T
    mqtt->data[n++] = 0x4;      //v.3.1.1
    mqtt->data[n++] = 0x2;      //Clean session
    mqtt->data[n++] = 0x0;      //Keep alive for 1 minute
    mqtt->data[n++] = 0x3c;     //Keep alive for 1 minute
    mqtt->data[n++] = 0x0;      //Payload length
    mqtt->data[n++] = 0x5;      //Payload length
    mqtt->data[n++] = 0x4b;      //K
    mqtt->data[n++] = 0x61;      //a
    mqtt->data[n++] = 0x76;      //v
    mqtt->data[n++] = 0x65;      //e
    mqtt->data[n++] = 0x6e;      //n

    mqttConnect = true;
    mqttLength = sizeof(uint8_t) + n;
}

void disconnectMqtt()
{
    uint8_t n = 0;

    mqttFrame *mqtt = mqttBuffer;
    mqtt->controlHeader = 0xE0; //Disconnect type, QoS 0
    mqtt->data[n++] = 0x00;     //Remaining length

    mqttDisconnectPending = true;
    mqttLength = sizeof(uint8_t) + n;
}

void publishMqtt(char strTopic[], char strData[])
{
    uint8_t n = 0;
    uint32_t varLength = 0, digit = 0;
    uint8_t i = 0;

    mqttFrame *mqtt = mqttBuffer;
    mqtt->controlHeader = 0x30; //Connect type, QoS 0
    varLength = strlen(strTopic) + strlen(strData) + 2; //Number of bytes in topic, payload and topic length (2 bytes)

    /*-- Algorithm from MQTT v3.1 protocol specification --*/
    do
    {
        digit = varLength % 128;
        varLength = varLength / 128;
        if(varLength > 0)
            digit = digit | 0x80;
        mqtt->data[n++] = digit;
    }
    while(varLength > 0);
    /*-----------------------------------------------------*/

    mqtt->data[n++] = (strlen(strTopic) >> 8) & 0xFF; //Topic length MSB
    mqtt->data[n++] = strlen(strTopic) & 0xFF;        //Topic length LSB
    while(strTopic[i] != NULL) //While not at the end count all chars
    {
        mqtt->data[n++] = strTopic[i];
        i++;
    }
    //Might need to send payload length
    i = 0;
    while(strData[i] != NULL) //While not at the end, count all chars
    {
        mqtt->data[n++] = strData[i];
        i++;
    }
    i = 0;

    mqttPublishPending = true;
    mqttLength = sizeof(uint8_t) + n;
}

void subscribeMqtt(char strTopic[])
{
    uint8_t n = 0;
    uint32_t varLength = 0, digit = 0;
    uint8_t i = 0;

    mqttFrame *mqtt = mqttBuffer;
    mqtt->controlHeader = 0x82; //Connect type, QoS 0, retain on
    varLength = strlen(strTopic) + 5;
    /*-- Algorithm from MQTT v3.1 protocol specification --*/
    do
    {
        digit = varLength % 128;
        varLength = varLength / 128;
        if(varLength > 0)
            digit = digit | 0x80;
        mqtt->data[n++] = digit;
        subVarLength++;
    }
    while(varLength > 0);
    /*-----------------------------------------------------*/
    mqtt->data[n++] = 0x00; //Message ID MSB = 0
    srand(random32());
    mqtt->data[n++] = subMsgID = (rand() % (255 - 1 + 1)) + 1; //Message ID LSB is random
    mqtt->data[n++] = (strlen(strTopic) >> 8) & 0xFF; //Topic length MSB
    mqtt->data[n++] = strlen(strTopic) & 0xFF;        //Topic length LSB
    while(strTopic[i] != NULL)
    {
        mqtt->data[n++] = strTopic[i];
        i++;
    }
    i = 0;
    mqtt->data[n++] = 0x00; //Requested QoS (0)

    mqttSubscribePending = true;
    mqttLength = sizeof(uint8_t) + n;
}

void unsubscribeMqtt(char strTopic[])
{
    uint8_t n = 0;
    uint32_t varLength = 0, digit = 0;
    uint8_t i = 0;

    mqttFrame *mqtt = mqttBuffer;
    mqtt->controlHeader = 0xA2; //Connect type, QoS 0
    varLength = strlen(strTopic) + 4;
    /*-- Algorithm from MQTT v3.1 protocol specification --*/
    do
    {
        digit = varLength % 128;
        varLength = varLength / 128;
        if(varLength > 0)
            digit = digit | 0x80;
        mqtt->data[n++] = digit;
        subVarLength++;
    }
    while(varLength > 0);
    /*-----------------------------------------------------*/
    mqtt->data[n++] = 0x00; //Message ID MSB = 0
    srand(random32());
    mqtt->data[n++] = unsubMsgID = (rand() % (255 - 1 + 1)) + 1; //Message ID LSB is random
    mqtt->data[n++] = (strlen(strTopic) >> 8) & 0xFF; //Topic length MSB
    mqtt->data[n++] = strlen(strTopic) & 0xFF;        //Topic length LSB
    while(strTopic[i] != NULL)
    {
        mqtt->data[n++] = strTopic[i];
        i++;
    }
    i = 0;

    mqttUnsubscribePending = true;
    mqttLength = sizeof(uint8_t) + n;
}

void pingMqtt()
{
    mqttFrame *mqtt = mqttBuffer;
    uint8_t n = 0;
    mqtt->controlHeader = 0xC0; //Ping type, QoS 0
    mqtt->data[n++] = 0x00;

    mqttPingReq = true;
    mqttLength = sizeof(uint8_t) + n;
}

void initialMqttSub()
{
    subscribeMqtt("remote");
}
