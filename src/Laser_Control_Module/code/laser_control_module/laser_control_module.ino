/*Laser Control Module
 * Benjamin Gillette
 * June 25, 2018
 * 
 * This program allows a computer to interface with a control module designed to
 * allow control of a surgical laser from that computer.  The control module is placed
 * in-line with an existing physical foot switch which originally controlled the laser
 * output to create a logical AND between the foot switch and the computer control;
 * that is, the laser output cannot be turned on unless both the physical foot switch
 * and the computer indicate that the laser output should be on.
 * 
 * During normal operation, the control computer sends commands to the Arduino over a
 * serial line indicating when to turn a relay on or off.  This relay is placed in
 * series with the signal line coming from the foot switch, which contains a normally
 * open switch that is closed when the pedal is pressed.  When both the foot switch and
 * the relay are closed, the foot switch signal line is connected to digital ground via
 * a line coming from the laser, indicating to the laser that the output should turn on.
 * 
 * In addition to the control relay, the Arduino also records input from the signal line
 * to determine when the laser is actually turned on.  When the state of the signal line
 * changes, this triggers an interrupt and the time at which this occurs is recorded and
 * sent back to the computer.  Additionaly, an LED is lit or extinguished to show the
 * current status of the signal line.
 * 
 * The laser foot switch has two parallel switches, each of which is controlled
 * independently in the manner described above.
 * 
 *                    Relays                    Foot switches                
 *                      |                             |                      
 *                      V                             V                      
 *    Ground ->  _____________________________________________               
 *                                                        |   |              
 *  Signal 1 ->  _______/ ____________________________/___|   |              
 *                                                            |              
 *  Signal 2 ->  _______/ ____________________________/_______|              
 *  
 *  (When not grounded, signal lines are pulled up to about +14V by the laser hardware)
 */

#include "laser_control_module.h"

//Global Variables
/////////////////////////
byte inputData[MESSAGE_LENGTH];   //serial input buffer
unsigned char messageRecieved = 0;  //input flag, set when a complete message has been recieved
volatile unsigned char inputChanged = 1; //Flag indicating that one of the signal lines has changed states
unsigned char bytesLeft = 0;  //# of bytes until message is complete
unsigned char laserState = 0; //Whether the laser is currently on or off
Switch switch1, switch2;
/////////////////////////

void setup(){
  pinMode(A1, INPUT);
  //Start serial communications
  Serial.begin(115200); //Open serial port for asynchronous communication w/ computer
  //Assign pins for the two switch lines (relay, input, LED).
  switch1.relay_pin = 9;
  switch1.input_pin = 2;
  switch1.indicator_pin = A3;
  switch2.relay_pin = 10;
  switch2.input_pin = 3;
  switch2.indicator_pin = A2;
  //Set pin I/O directions
  pinMode(switch1.relay_pin, OUTPUT);
  pinMode(switch1.input_pin, INPUT);
  pinMode(switch1.indicator_pin, OUTPUT);
  pinMode(switch2.relay_pin, OUTPUT);
  pinMode(switch2.input_pin, INPUT);
  pinMode(switch2.indicator_pin, OUTPUT);
  //Set up interrupts for detecting changes in input
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(switch1.input_pin), switch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(switch2.input_pin), switch2ISR, CHANGE);
  interrupts();
  //Start with switches open (laser off).
  digitalWrite(switch1.relay_pin, 0);
  digitalWrite(switch2.relay_pin, 0);
}

void loop(){
  Message inputMessage; //this will hold the recieved message (if we've recieved one)
  if(messageRecieved){
    messageRecieved = 0;  //clear flag
    inputMessage = unpackMessage(inputData);  //unpack message
    //Parse data from message (see arduino_serial.h for message structure)
    //If the message was an output command, set the relay outputs to the state specified in the message.
    if(inputMessage.isOutput){
      digitalWrite(switch1.relay_pin, inputMessage.data);
      digitalWrite(switch2.relay_pin, inputMessage.data);
    }
    //If the recieved message was a read message, read in the state of the laser signal lines and send back the result
    else
      sendData(laserState);
  }

  //If the input has changed, check whether the laser output has changed and send an update if it has.
  if(inputChanged){
    inputChanged = 0;
    //check the states of the two input pins to determine whether the resulting laser output has changed
    unsigned char switch1State = !digitalRead(switch1.input_pin);
    unsigned char switch2State = !digitalRead(switch2.input_pin);
    //Since we've just checked the output state of each switch, this is a good time to update the indicator lights for each switch.
    digitalWrite(switch1.indicator_pin, switch1State); //Light indicates when line is pulled low, so the state is inverted.
    digitalWrite(switch2.indicator_pin, switch2State);
    //If both signal lines are pulled low, the laser is on; otherwise, it is off.
    unsigned char newState = switch1State && switch2State;
    if(laserState != newState){
      laserState = newState;
      sendData(laserState);
    }
  }
  
  //Check the serial buffer to see if a message has been recieved.  If so, read it in and set a flag.
  while(Serial.available()){
    //Read in new byte
    byte inputByte = (byte) Serial.read();
    //if a start byte is recieved and we're not in the middle of a message, start recieving.
    if((inputByte == START_BYTE) && (!bytesLeft)){
      bytesLeft = MESSAGE_LENGTH;
    }
    
    //if we're currently recieving, record the new byte.
    if(bytesLeft){
      inputData[MESSAGE_LENGTH - bytesLeft] = inputByte;
      bytesLeft--;
      //if that was the last byte in the message, flag that a message is available.
      if(!bytesLeft)
        messageRecieved = 1;
    }
  }
}

//Interrupt service routines
void switch1ISR(){
  inputChanged = 1;
}
void switch2ISR(){
  inputChanged = 1;
}

