#ifndef LCM_H
#define LCM_H

#include <Arduino.h>
#include "arduino_serial.h"

/*Structure to store all of the pin connections associated with a particular switch.
 * relay_pin: output controlling the relay on this switch line.
 * input_pin: input showing the current status of this switch line.
 * indicator_pin: output to an LED that shows the status of the input line.
 */
typedef struct{
  unsigned char relay_pin;  //Pin controlling the relay coil; output
  unsigned char input_pin;  //Pin sensing the state of the signal line switched by this relay; input
  unsigned char indicator_pin;  //Pin controlling an LED that will be set to show the state of the signal line switched by this relay.
} Switch;

//Function Prototypes
void switch1ISR(void);  //Interrupt service routine run when the state on the switch 1 signal line changes.
void switch2ISR(void);  //Interrupt service routine run when the state on the switch 2 signal line changes.
#endif
