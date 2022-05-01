#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    IrSender.begin(); // Start with IR_SEND_PIN as send pin digital 3
}

int servos = 5;
int* vals = new int[servos];
int analogPins[] = {A3, A4, A5, A6, A7};
uint32_t data=0;

void loop() {

    // read values from analog pins
    for(int i=0; i<servos; i++)
      vals[i] = map(analogRead(analogPins[i]), 0, 1024, 0, 63);

    // insert the command, it can be 00, 01, 10, 11
    String Final = "00";

    // build the binary string containing the 5 values of servo readings 6 bits for each, and 2 bits for command
    for(int j=0; j<servos; j++)
      for(int i = 5; i >= 0; i--) // don't change this from 5 it belongs to 6 bits not amount of servos
        Final += String((vals[j]>>i)&1);

    // convert the string into uint32_t
    data = 0;
    for (int i=0; i< Final.length(); i++) {
      data *= 2;
      if (Final[i] == '1') data++;
    }

    // send the data
    digitalWrite(LED_BUILTIN, HIGH);
    IrSender.sendNECRaw(data, 0); // 0 is the amount of times we wanna resend this report, just do it once wkhls
    
    delay(100);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
    digitalWrite(LED_BUILTIN, LOW);
}
