#ifndef ARDUINO_SERIAL_H
#define ARDUINO_SERIAL_H
/*
 * Serial packet structure:
 * X = don't care, b = data bit
 *    +----------------Start byte (ASCII: new line, 'A')
 *    |           +----Read/write (0/1)
 * +--+---+       |+---Laser off/on (0/1)
 * |      |       ||
 * 01000001/XXXXXXbb
 */

#include <Arduino.h>

#define MESSAGE_LENGTH 2  //message length in bytes
#define START_BYTE 'A'   //serial message start byte

//structure to hold decoded message data
typedef struct{
    byte isOutput;  //Whether the message is a command to set the output or a request for the output state
    int8_t data;  //If the message is an output command, this is what the output should be set to.
} Message;

//Function prototypes
Message unpackMessage(byte rawMessage[]); //Decodes an array of bytes into a struct containing the relevant message data.
uint16_t calculateChecksum(byte message[]); //Given a data packet, returns the CRC16 checksum for that packet.
void sendData(byte state);  //Given a component, data packet, and I/O direction, packs the data and sends it over the serial port.
#endif
