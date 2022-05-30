/**
 * A simple test-sketch that uses the IRemote library to detect when the power on/off button
 * is pressed on an NEC remote control. If a button press is detected, the sketch toggles
 * the on-board state of the LED (pin 13 on the Arduino Uno).
 */

#include <IRremote.h>

#define IR_RECEIVE_PIN 2

IRrecv receiver(IR_RECEIVE_PIN);

void setup()
{
  // There's no need to set up the IR_RECEIVE_PIN with pinMode
  // the library takes care of that...
  // pinMode(IR_RECEIVE_PIN, INPUT);
 

  Serial.begin(9600);

  while(!Serial)
  {  }

   Serial.println("IR Receive test");
   IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop()
{
   if (IrReceiver.decode())
   {
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      IrReceiver.resume();
   }
}
