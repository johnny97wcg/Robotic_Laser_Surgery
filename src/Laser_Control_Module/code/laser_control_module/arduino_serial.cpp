#include "arduino_serial.h"

//Decodes an array of bytes into a struct containing the relevant message data.
//(See arduino_serial.h for message structure)
Message unpackMessage(byte rawMessage[]){
  Message unpackedMessage;
  unpackedMessage.isOutput = (rawMessage[1] & 0b00000010) >> 1;
  unpackedMessage.data = rawMessage[1] & 0b00000001;
  return unpackedMessage;
}

//Given a component, data packet, and I/O direction, packs the data and sends it over the serial port.
void sendData(byte state){
  //First, pack the data into an array of bytes
  byte outputData[MESSAGE_LENGTH];
  outputData[0] = START_BYTE;
  outputData[1] = state;
  //Then, send the message over the serial port.
  for(int x = 0; x < MESSAGE_LENGTH; x++)
    Serial.write(outputData[x]);
}
