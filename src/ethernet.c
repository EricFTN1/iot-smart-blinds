// Ethernet Framework for Projects 1 and 2
// Spring 2025
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller on SPI0
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS (SW controlled) on PA3
//   WOL on PB3
//   INT on PC6

// Pinning for IoT projects with wireless modules:
// N24L01+ RF transceiver
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS on PE0
//   INT on PB2
// Xbee module
//   DIN (UART1TX) on PC5
//   DOUT (UART1RX) on PC4

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "eeprom.h"
#include "gpio.h"
#include "spi0.h"
#include "uart0.h"
#include "wait.h"
#include "timer.h"
#include "eth0.h"
#include "arp.h"
#include "ip.h"
#include "icmp.h"
#include "udp.h"
#include "tcp.h"
#include "mqtt.h"

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4
#define MOTOR1_GPIO PORTE,1
#define MOTOR2_GPIO PORTE,2

// EEPROM Map
#define EEPROM_DHCP        1
#define EEPROM_IP          2
#define EEPROM_SUBNET_MASK 3
#define EEPROM_GATEWAY     4
#define EEPROM_DNS         5
#define EEPROM_TIME        6
#define EEPROM_MQTT        7
#define EEPROM_REMOTETABLE 8
#define EEPROM_REMOTEINDEX 512
#define EEPROM_ERASED      0xFFFFFFFF

#define MAX_REMOTES 30

extern bool isMqttConnected;
#define IR_Sensor_Mask 4

typedef struct remote
{
    uint16_t remoteAdd;
    uint16_t remoteData;
    char topic[30];
    char data[30];
} REMOTE;
REMOTE currRemote;
uint32_t *remotePtr;
uint16_t remoteTableIndex = 0;
uint8_t REMOTE_ENTRY_SIZE = (sizeof(REMOTE) / 4 + ((sizeof(REMOTE) % 4) ? 1 : 0));
uint32_t code = 0, IR_Code = 0;
char mqttTopic[50];
bool timerFlag, learningMode = false, learningMode2 = false, found = false;
//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(MOTOR1_GPIO);
    selectPinPushPullOutput(MOTOR2_GPIO);
    selectPinDigitalInput(PUSH_BUTTON);
    enablePinPullup(PUSH_BUTTON);

    // Configuring a Wide timer for the IR sensor (TV remote) and a narrow timer for counting the 1's and 0's from the remote.
    GPIO_PORTD_DEN_R |=    IR_Sensor_Mask;          // Digitally Enable PC6
    GPIO_PORTD_AFSEL_R |=  IR_Sensor_Mask;          // Alternative function select PC6
    GPIO_PORTD_PCTL_R &=  ~GPIO_PCTL_PD2_M;         // Turning off default functionality
    GPIO_PORTD_PCTL_R |=   GPIO_PCTL_PD2_WT3CCP0;   // Turning on Wide Timer Functionality

    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;               // turn-off timer/counter before changing it.
    WTIMER3_CFG_R = 4;                              // Configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR |TIMER_TAMR_TAMR_CAP; // Configure in edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_NEG;          // Measure time from negative edge to negative edge
    WTIMER3_TAV_R = 0;                              // Set the timer value to 0
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                // Turn on interrupts
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                // Turn on the timer
    NVIC_EN3_R = 1 << (INT_WTIMER3A -16-96);        // Turn on interrupt 116 (WTIMER3A)

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;          // configure as 32-bit timer (A)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;         // configure for one-shot mode
    TIMER2_TAILR_R = 272e4;                         // set load value to 272e4
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                // Turn on the timer interrupt mask
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);             // turn-on interrupt 39 (TIMER2A)
}

// Narrow timer to count exactly how long we are allowed to read bits for the remote control
void narrowTimer2ISR()
{
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
    TIMER2_TAILR_R = 272e4;
    timerFlag = true;
    IR_Code = code;
    code = 0;
}
// ISR to detect a GPIO interrupt from the remote and process the data coming in
void wideTimer3AISR()
{
    static int8_t state = 0;
    if(timerFlag == true)
        state = 0;
    if(state == 0)
    {
        WTIMER3_TAV_R = 0;
        timerFlag = false;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        state = 1;
    }
    if(state == 1 && (WTIMER3_TAV_R >= 520000 && WTIMER3_TAV_R <= 560000 )) // If we see the initial negative edge time, we know we're getting the right signal
    {
        WTIMER3_TAV_R = 0;
        state = 2;
    }
    if(state == 2 && ((WTIMER3_TAV_R > 33750 && WTIMER3_TAV_R < 56250) || // If we see either a 1 or a 0 worth of time, we are getting the right signal
                       WTIMER3_TAV_R > 78750 && WTIMER3_TAV_R < 101250))
    {
        if(!timerFlag)
        {
            code = code << 1;
            if(WTIMER3_TAV_R > 67500) // If in the range of a 1, mask a 1 into the LSB, otherwise do nothing (AKA a 0)
            {
                code |= 1;
            }
            WTIMER3_TAV_R = 0;
        }
    }
    WTIMER3_ICR_R |= TIMER_ICR_CAECINT;
}
void displayRemoteTable(void)
{
    putsUart0("  REMOTES:\n");

    REMOTE r;
    uint32_t *dataPtr = (uint32_t *)&r;
    uint16_t entry = 0;
    uint32_t firstWord = 0;
    char str[64];
    for (entry = 0; entry < MAX_REMOTES; entry++)
    {
        uint16_t addr = EEPROM_REMOTETABLE + (entry * REMOTE_ENTRY_SIZE);

        // Check if first word is EEPROM_ERASED
        firstWord = readEeprom(addr);
        if (firstWord == EEPROM_ERASED)
            continue;  // Skip erased entries

        // Read entire REMOTE struct
        dataPtr[0] = firstWord;
        uint16_t i = 1;
        for (i = 1; i < REMOTE_ENTRY_SIZE; i++)
            dataPtr[i] = readEeprom(addr + i);

        // Display it
        snprintf(str, sizeof(str), "    0x%04X%04X", r.remoteAdd, r.remoteData);
        putsUart0(str);
        putsUart0(" | Topic: ");
        putsUart0(r.topic);
        putsUart0(" | Data: ");
        putsUart0(r.data);
        putcUart0('\n');
    }
    firstWord = readEeprom(EEPROM_REMOTEINDEX);
    snprintf(str, sizeof(str), "Index: %04d \n Eeprom: %04d", firstWord/16, firstWord);
    putsUart0(str);
    putcUart0('\n');
}

void clearRemoteTable(void)
{
    uint16_t i = 0;
    while(readEeprom(EEPROM_REMOTETABLE + i) != EEPROM_ERASED)
    {
        writeEeprom(EEPROM_REMOTETABLE + i, EEPROM_ERASED);
        i++;
    }
    writeEeprom(EEPROM_REMOTEINDEX, 0);
}


void displayConnectionInfo()
{
    uint8_t i;
    char str[20];
    uint8_t mac[6];
    uint8_t ip[4];
    getEtherMacAddress(mac);
    putsUart0("  HW:    ");
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%02"PRIu8, mac[i]);
        putsUart0(str);
        if (i < HW_ADD_LENGTH-1)
            putcUart0(':');
    }
    putcUart0('\n');
    getIpAddress(ip);
    putsUart0("  IP:    ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    getIpSubnetMask(ip);
    putsUart0("  SN:    ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    getIpGatewayAddress(ip);
    putsUart0("  GW:    ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    getIpDnsAddress(ip);
    putsUart0("  DNS:   ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    getIpTimeServerAddress(ip);
    putsUart0("  Time:  ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    getIpMqttBrokerAddress(ip);
    putsUart0("  MQTT:  ");
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        snprintf(str, sizeof(str), "%"PRIu8, ip[i]);
        putsUart0(str);
        if (i < IP_ADD_LENGTH-1)
            putcUart0('.');
    }
    putcUart0('\n');
    if (isEtherLinkUp())
        putsUart0("  Link is up\n");
    else
        putsUart0("  Link is down\n");
}

void open_blinds(void)
{
    setPinValue(MOTOR1_GPIO, 0);
    setPinValue(MOTOR2_GPIO, 1);
}

void close_blinds(void)
{
    setPinValue(MOTOR1_GPIO, 1);
    setPinValue(MOTOR2_GPIO, 0);
}

void stop_blinds(void)
{
    setPinValue(MOTOR1_GPIO, 0);
    setPinValue(MOTOR2_GPIO, 0);
}

void readConfiguration()
{
    uint32_t temp;
    uint8_t* ip;

    temp = readEeprom(EEPROM_IP);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpAddress(ip);
    }
    temp = readEeprom(EEPROM_SUBNET_MASK);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpSubnetMask(ip);
    }
    temp = readEeprom(EEPROM_GATEWAY);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpGatewayAddress(ip);
    }
    temp = readEeprom(EEPROM_DNS);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpDnsAddress(ip);
    }
    temp = readEeprom(EEPROM_TIME);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpTimeServerAddress(ip);
    }
    temp = readEeprom(EEPROM_MQTT);
    if (temp != EEPROM_ERASED)
    {
        ip = (uint8_t*)&temp;
        setIpMqttBrokerAddress(ip);
    }
    REMOTE r;
    uint32_t *dataPtr = (uint32_t *)&r;

    uint16_t wordCount = sizeof(REMOTE) / 4;
    if (sizeof(REMOTE) % 4 != 0) wordCount += 1;

    // Check if the first word is valid
    temp = readEeprom(EEPROM_REMOTETABLE);
    // Read the full struct from EEPROM

    uint16_t i = 1;
    for (i = 1; i < wordCount; i++)
    {
    dataPtr[i] = readEeprom(EEPROM_REMOTETABLE + i);
    }
}

void setRemote(REMOTE *r, uint16_t remoteAdd, uint16_t remoteData, char topic[], char data[])
{
    if (r == NULL) return; // Safety check

    r->remoteAdd = remoteAdd;
    r->remoteData = remoteData;

    // Copy topic and data strings safely
    strncpy(r->topic, topic, sizeof(r->topic) - 1);
    r->topic[sizeof(r->topic) - 1] = '\0'; // Ensure null-termination

    strncpy(r->data, data, sizeof(r->data) - 1);
    r->data[sizeof(r->data) - 1] = '\0'; // Ensure null-termination
}

#define MAX_CHARS 80
char strInput[MAX_CHARS+1];
char* token;
uint8_t count = 0;

uint8_t asciiToUint8(const char str[])
{
    uint8_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hhx", &data);
    else
        sscanf(str, "%hhu", &data);
    return data;
}

void processShell()
{
    bool end;
    char c;
    uint8_t i;
    uint8_t ip[IP_ADD_LENGTH];
    uint32_t* p32;
    char str[50];
    char connected[20];
    char *topic, *data;

    if (kbhitUart0())
    {
        c = getcUart0();

        end = (c == 13) || (count == MAX_CHARS);
        if (!end)
        {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                strInput[count++] = c;
        }
        else
        {
            strInput[count] = '\0';
            count = 0;
            token = strtok(strInput, " ");
            if (strcmp(token, "tcp") == 0)
            {
                token = strtok(NULL, " ");
                if(strcmp(token, "connect") == 0)
                {
                    connectTcp();
                }
                if(strcmp(token, "disconnect") == 0)
                {
                    disconnectTcp();
                }
            }
            if (strcmp(token, "mqtt") == 0)
            {
                token = strtok(NULL, " ");
                if (strcmp(token, "connect") == 0)
                {
                    connectMqtt();
                }
                if (strcmp(token, "disconnect") == 0)
                {
                    disconnectMqtt();
                }
                if (strcmp(token, "publish") == 0)
                {
                    topic = strtok(NULL, " ");
                    data = strtok(NULL, " ");
                    if (topic != NULL && data != NULL)
                        publishMqtt(topic, data);
                }
                if (strcmp(token, "subscribe") == 0)
                {
                    topic = strtok(NULL, " ");
                    if (topic != NULL)
                        subscribeMqtt(topic);
                }
                if (strcmp(token, "unsubscribe") == 0)
                {
                    topic = strtok(NULL, " ");
                    if (topic != NULL)
                        unsubscribeMqtt(topic);
                }
            }
            if(learningMode && (strcmp(token, "remote") == 0))
            {
                char *remoteTopic;
                char *remoteData;

                remoteTopic = strtok(NULL, " ");
                remoteData = strtok(NULL, " ");
                strcpy(currRemote.topic, remoteTopic);
                strcpy(currRemote.data, remoteData);
                putsUart0("\n Now Press the button to set it\n");
                learningMode = false; learningMode2 = true;

                memset(remoteTopic, 0, sizeof(remoteTopic));
                memset(remoteData, 0, sizeof(remoteData));
            }
            if (strcmp(token, "ip") == 0)
            {
                displayConnectionInfo();
            }
            if (strcmp(token, "ping") == 0)
            {
                for (i = 0; i < IP_ADD_LENGTH; i++)
                {
                    token = strtok(NULL, " .");
                    ip[i] = asciiToUint8(token);
                }
                //removed from this version to save space: sendPingRequest(ip)
            }
            if (strcmp(token, "reboot") == 0)
            {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }
            if (strcmp(token, "remoteTable") == 0)
            {
                displayRemoteTable();
            }
            if (strcmp(token, "remoteTableClear") == 0)
            {
                clearRemoteTable();
            }
            if (strcmp(token, "set") == 0)
            {
                token = strtok(NULL, " ");
                if (strcmp(token, "ip") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpAddress(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_IP, *p32);
                }
                if (strcmp(token, "remoteIndex") == 0)
                {
                    uint8_t temp =0;
                    token = strtok(NULL, " ");
                    temp = asciiToUint8(token);
                    writeEeprom(EEPROM_REMOTEINDEX, temp*16);
                }
                else if (strcmp(token, "remote") == 0)
                {
                    char *hexStr = strtok(NULL, " ");
                    char *topic = strtok(NULL, " ");
                    char *data = strtok(NULL, " ");
                    uint32_t indexRemote = readEeprom(EEPROM_REMOTEINDEX);
                    if (indexRemote>512)
                    {
                        writeEeprom(EEPROM_REMOTEINDEX, 0);
                        putsUart0("Index too high reset to 0\n");
                    }
                    if (hexStr && topic && data)
                    {
                        unsigned int hexValue;
                        if (sscanf(hexStr, "%x", &hexValue) == 1)
                        {
                            uint16_t remoteAdd = (hexValue >> 16) & 0xFFFF;
                            uint16_t remoteData = hexValue & 0xFFFF;

                            REMOTE r;
                            setRemote(&r, remoteAdd, remoteData, topic, data);

                            uint32_t *dataPtr = (uint32_t *)&r;

                            // How many 32-bit words does REMOTE occupy?
                            uint16_t wordCount = sizeof(REMOTE) / 4;
                            if (sizeof(REMOTE) % 4 != 0) wordCount += 1; // Account for padding if not divisible

                            // Write struct to EEPROM
                            uint16_t i = 0;

                            uint16_t addr = EEPROM_REMOTETABLE + indexRemote;

                            for (i = 0; i < REMOTE_ENTRY_SIZE; i++)
                            {
                                writeEeprom(addr + i, dataPtr[i]);

                            }
                            writeEeprom(EEPROM_REMOTEINDEX, indexRemote + REMOTE_ENTRY_SIZE);
                        }
                    }
                }
                if (strcmp(token, "sn") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpSubnetMask(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_SUBNET_MASK, *p32);
                }
                if (strcmp(token, "gw") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpGatewayAddress(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_GATEWAY, *p32);
                }
                if (strcmp(token, "dns") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpDnsAddress(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_DNS, *p32);
                }
                if (strcmp(token, "time") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpTimeServerAddress(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_TIME, *p32);
                }
                if (strcmp(token, "mqtt") == 0)
                {
                    for (i = 0; i < IP_ADD_LENGTH; i++)
                    {
                        token = strtok(NULL, " .");
                        ip[i] = asciiToUint8(token);
                    }
                    setIpMqttBrokerAddress(ip);
                    p32 = (uint32_t*)ip;
                    writeEeprom(EEPROM_MQTT, *p32);
                }
            }
            if(strcmp(token, "status") == 0)
            {
                snprintf(str, sizeof(str), "TCP state: %d\n", getTcpState(0));
                putsUart0(str);

                if(isMqttConnected)
                    strcpy(connected, "connected");
                else
                    strcpy(connected, "disconnected");
                snprintf(str, sizeof(str), "MQTT Connection Status: %s\n", connected);
                putsUart0(str);
            }
            if (strcmp(token, "help") == 0)
            {
                putsUart0("Commands:\n");
                putsUart0("  mqtt ACTION [USER [PASSWORD]]\n");
                putsUart0("    where ACTION = {connect|disconnect|publish TOPIC DATA\n");
                putsUart0("                   |subscribe TOPIC|unsubscribe TOPIC}\n");
                putsUart0("  ip\n");
                putsUart0("  ping w.x.y.z\n");
                putsUart0("  reboot\n");
                putsUart0("  set ip|gw|dns|time|mqtt|sn w.x.y.z\n");
            }
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

// Max packet is calculated as:
// Ether frame header (18) + Max MTU (1500)
#define MAX_PACKET_SIZE 1518

int main(void){
    uint8_t buffer[MAX_PACKET_SIZE];
    etherHeader *data = (etherHeader*) buffer;
    socket s;
    char str[50];

    // Init controller
    initHw();

    // Setup UART0
    initUart0();
    setUart0BaudRate(115200, 40e6);

    // Init timer
    initTimer();

    // Init sockets
    initSockets();

    // Init ethernet interface (eth0)
    putsUart0("\nStarting eth0\n");
    initEther(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
    setEtherMacAddress(2, 3, 4, 5, 6, 139);


    // Init EEPROM
    initEeprom();
    readConfiguration(); //Will read the command line, this is where we type in our IP address

    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);

    // Init Tcp Ports
    initTCP();
    connectTcp();
//    startOneshotTimer(connectMqtt, 1);
    connectMqtt();

    //Create Socket
    uint8_t emptyMsg[0];
    bool initialSub = true;
    // Main Loop
    // RTOS and interrupts would greatly improve this code,
    // but the goal here is simplicity
    while (true)
    {
        /* LG 55UH8509 remote codes */
        if(IR_Code)
        {
            uint32_t reading = 0;
            snprintf(str, sizeof(str), "Remote Add:%x\tRemote Data:%x\n", (IR_Code >> 16), (IR_Code & 0x0000FFFF));
            putsUart0(str);
            if(IR_Code == 0x20DF1EE1)
            {
                putsUart0("Learning mode\n\nPlease enter remote <topic> <data>\n");
                learningMode = true;
            }
            else if(learningMode2)
            {
                currRemote.remoteAdd = IR_Code >> 16;
                currRemote.remoteData = IR_Code & 0xFFFF;
                REMOTE r;
                uint32_t *dataPtr = (uint32_t*)&currRemote;
                uint32_t *dataPtr2 = (uint32_t*)&r;
                uint16_t entry = 0;
                uint16_t i = 0;
                uint32_t indexRemote = readEeprom(EEPROM_REMOTEINDEX);
                uint16_t wordCount = sizeof(REMOTE) / 4;
                if (sizeof(REMOTE) % 4 != 0) wordCount += 1; // padding

                bool found = false;
                uint16_t addr = EEPROM_REMOTETABLE; // Start of table

                // Search existing entries
                for (entry = 0; entry < indexRemote; entry += wordCount)
                {
                    for (i = 0; i < wordCount; i++)
                    {
                        dataPtr2[i] = readEeprom(addr + entry + i); // Read current remote struct
                    }
                    uint32_t temp = (r.remoteAdd << 16) | r.remoteData;
                    if (temp == IR_Code)
                    {
                        // Found, overwrite it
                        for (i = 0; i < wordCount; i++)
                        {
                            writeEeprom(addr + entry + i, dataPtr[i]);
                        }
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    // Append at the end
                    for (i = 0; i < wordCount; i++)
                    {
                        writeEeprom(addr + indexRemote + i, dataPtr[i]);
                    }
                    // Update index
                    writeEeprom(EEPROM_REMOTEINDEX, indexRemote + wordCount);
                }

                learningMode2 = false;
            }
//            else if(learningMode2)
//            {
//                currRemote.remoteAdd = IR_Code >> 16;
//                currRemote.remoteData = IR_Code & 0xFFFF;
//                REMOTE r;
//                uint32_t *dataPtr = (uint32_t *)&currRemote;
//                uint32_t *dataPtr2 = (uint32_t *)&r;
//                uint32_t indexRemote = readEeprom(EEPROM_REMOTEINDEX);
//                // How many 32-bit words does REMOTE occupy?
//                uint16_t wordCount = sizeof(REMOTE) / 4;
//                bool found = false;
//                if (sizeof(REMOTE) % 4 != 0) wordCount += 1; // Account for padding if not divisible
//                uint32_t temp = 0;
//                // Write struct to EEPROM
//                uint16_t i = 0;
//
//                uint16_t addr = EEPROM_REMOTETABLE + indexRemote;
//
//                for (i = 0; i < REMOTE_ENTRY_SIZE; i++)
//                {
//                    dataPtr2[i] = readEeprom(EEPROM_REMOTETABLE + i);
//                    temp = (r.remoteAdd << 16) | r.remoteData;
//                     if(temp == IR_Code)
//                     {
//                         writeEeprom(EEPROM_REMOTETABLE + i, dataPtr2[i]);
//                         break;
//                     }
//                     else if(!found)
//                     {
//                         writeEeprom(addr + i, dataPtr[i]);
//                     }
//                }
//                writeEeprom(EEPROM_REMOTEINDEX, indexRemote + REMOTE_ENTRY_SIZE);
//                learningMode2 = false;
//            }
            else
            {
                REMOTE r;
                uint32_t *dataPtr = (uint32_t *)&r;
                uint16_t entry = 0;
                uint32_t firstWord = 0;
                char str[64];
                for (entry = 0; entry < MAX_REMOTES; entry++)
                {
                    uint16_t addr = EEPROM_REMOTETABLE + (entry * REMOTE_ENTRY_SIZE);
                    // Check if first word is EEPROM_ERASED
                    firstWord = readEeprom(addr);
                    if (firstWord == EEPROM_ERASED)
                        continue;  // Skip erased entries

                    // Read entire REMOTE struct
                    dataPtr[0] = firstWord;
                    uint16_t i = 1;
                    uint32_t temp = 0;
                    for (i = 1; i < REMOTE_ENTRY_SIZE; i++)
                    {
                        dataPtr[i] = readEeprom(addr + i);
                        temp = (r.remoteAdd << 16) | r.remoteData;
                         if(temp == IR_Code)
                         {
                             found = true;
                         }
                    }
                if(found == true)
                {
                    publishMqtt(r.topic, r.data);
                    found = false;
                    break;

                }

                memset(r.topic, 0, sizeof(r.topic));
                memset(r.data, 0, sizeof(r.data));
                }
            }
            IR_Code = 0;
        }


//        switch(IR_Code)
//        {
//            case 0x20DFE01F: putsUart0("Left Button\n");
//                             IR_Code = 0;
//                             if(isMqttConnected)
//                                 publishMqtt("remote", "left_on");
//                             break;
//            case 0x20DF609F: putsUart0("Right Button\n");
//                             IR_Code = 0;
//                             if(isMqttConnected)
//                                 publishMqtt("remote", "right_on");
//                             break;
//            case 0x20DF8D72: putsUart0("Stop Button\n");
//                             IR_Code = 0;
//                             if(isMqttConnected)
//                                 publishMqtt("remote", "stop");
//                             break;
//        }
        // Terminal processing here
        processShell();

        // TCP pending messages (SEND THE FIRST MESSAGE WITH THE SYN)
        // I need to remember to do something later based on a message I received
        sendTcpPendingMessages(data);

        // Packet processing
        if (isEtherDataAvailable())
        {
            if (isEtherOverflow())
            {
                setPinValue(RED_LED, 1);
                waitMicrosecond(100000);
                setPinValue(RED_LED, 0);
            }

            // Get packet
            getEtherPacket(data, MAX_PACKET_SIZE);

            // Handle ARP request
            if (isArpRequest(data))
                sendArpResponse(data);

            if (isArpResponse(data))
                processTcpArpResponse(data);


            // Handle IP datagram
            if (isIp(data))
            {
            	if (isIpUnicast(data))
            	{
                    // Handle ICMP ping request
                    if (isPingRequest(data))
                    {
                        sendPingResponse(data);
                    }

                    // Handle TCP datagram
                    if (isTcp(data))
                    {
                        //Go to your web browser, type in your ip address, turn on an LED
                        setPinValue(RED_LED, 1);
                        // Is this part of a conversation that I've already been happening? Is the port already open?
                        // If the port is open, process the message, otherwise acknowledge the attempt and sent reset message
                        if (isTcpPortOpen(data))
                        {
                            processTcpResponse(data);
                        }
                        // EX: I'm not a web server, send reset back
                        else
                            sendTcpMessage(data, &s, ACK | RST, emptyMsg, 0);
                    }
                }
            }
        }
    }
}

