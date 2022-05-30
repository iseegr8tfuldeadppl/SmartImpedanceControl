#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

void setup() {
    Serial.begin(115200);
    IrSender.begin(); // Start with IR_SEND_PIN as send pin and if NO_LED_FEEDBACK_CODE is NOT defined, enable feedback LED at default feedback LED pin
}

int servos = 5;
int* vals = new int[servos];
uint32_t result;

void loop() {
    
    // Results for the first loop to: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    //IrSender.sendNEC(0x102, 0x34, 1);

    vals[0] = 63;
    vals[1] = 50;
    vals[2] = 0;
    vals[3] = 11;
    vals[4] = 50;

    String Final = "00";
    for(int j=0; j<5; j++){
      if(vals[j]>63) vals[j] = 63;
      for(int i = 6-1; i >= 0; i--){
        Final += String((vals[j]>>i)&1);
      }
    }

    uint32_t number=0;
    for (int i=0; i< Final.length(); i++)  // for every character in the string  strlen(s) returns the length of a char array
    {
      number *= 2; // double the result so far
      if (Final[i] == '1') number++;  //add 1 if needed
    }

    Serial.print("Sent: ");
    Serial.println(number, HEX);




    String receivedString = "";
    uint32_t received = 0x3FC802F2;
    for(int j=5; j>0; j--){
      String val = "";
      for(int i = 6*j-1; i >= 6*j-6; i--){
        Serial.print(" " + String((received>>i)&1));
        val += String((received>>i)&1);
      }

      uint8_t valval=0;
      for (int i=0; i< val.length(); i++)  // for every character in the string  strlen(s) returns the length of a char array
      {
        valval *= 2; // double the result so far
        if (val[i] == '1') valval++;  //add 1 if needed
      }
      
      Serial.println(" Received " + String(valval));
    }

      String command = String((received>>30)&1) + String((received>>31)&1);
      Serial.println(" Command " + command);
    


    
    //IrSender.sendNECRaw(0x, 0);
    
    delay(1000);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}
