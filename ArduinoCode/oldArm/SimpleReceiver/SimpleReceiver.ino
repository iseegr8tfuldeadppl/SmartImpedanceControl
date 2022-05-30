#include <Servo.h>
#include <EEPROM.h>
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

#define DECODE_NEC          // Includes Apple and Onkyo


Servo servo, servo2;
long value;
long target_angle = 90;
long current_angle = 90;
int sensorTooMuch = 20;

long target2_angle = 90;
long current2_angle = 90;

String* msgParts;
String msg;
int Step = 1;
int servos = 5;
uint8_t readings[] = {0, 0, 0, 0, 0};
String receivedString;
uint32_t received;
String command;


void setup() {
  Serial.begin(115200);
  /*
  while(!Serial){}

  // prepare hardware
  servo.attach(9);
  //servo2.attach(6);

  // load data from eeprom
  float eeprom_test;
  EEPROM.get( 0, eeprom_test );
  boolean force_pull_from_eeprom = false;
  if (eeprom_test!=6.9 || force_pull_from_eeprom) {
    EEPROM.put( 0, 6.9 );
    EEPROM.put( 10, target_angle );
    EEPROM.put( 20, sensorTooMuch );
    EEPROM.put( 30, target2_angle );
    
    Serial.println("Ok, first time launch, servo value: " + String(target_angle));
  } else {
    EEPROM.get( 10, target_angle );
    EEPROM.get( 20, sensorTooMuch );
    EEPROM.get( 30, target2_angle );

    Serial.println("Ok, loaded previous servo value: " + String(target_angle));
  }

  // update hardware
  servo.write(target_angle);
  servo2.write(target2_angle);
  */

  pinMode(LED_BUILTIN, OUTPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}


void loop() {

  // IR communication with the measuring arduino
  if (IrReceiver.decode()) {

    // retrieve the 32 bit binary msg
    received = IrReceiver.decodedIRData.decodedRawData;

    // convert that binary number to string
    receivedString = "";
    for(int i = 31; i >=0; i--)
      receivedString += String((received>>i)&1);

    // loop over all five motor readings
    for(int j=0; j<servos; j++){

      // take the current motor's portion of the 32bit binary number
      String valString = receivedString.substring(6*j+2, 6*j+2+6);
      
      // reset current readings
      readings[j] = 0;
      
      // convert that reading into an 8 bit integer
      for (int i=0; i< valString.length(); i++) {
        readings[j] *= 2; // double the result so far
        if (valString[i] == '1') readings[j]++;  //add 1 if needed
      }

    }

    // retrieve command
    //command = String(receivedString[0]) + String(receivedString[1]);

      String toPrint = "Currents: ";
      for(int i=0; i<servos; i++){
        toPrint += String(readings[i]) + " ";
      }
      Serial.println(toPrint);
      
    IrReceiver.resume(); // Enable receiving of the next value
  }

  /*
  // Communication: reply to commands from user
  if(Serial.available()){
    msg = Serial.readStringUntil('\n');
    msgParts = getValues(msg);
  
    if(msgParts[0]=="ANGLE"){ // ANGLE <servo number> <integer angle value>
      value = msgParts[2].toInt();
    
      Serial.println("before");
      if(!isnan(value)){
        
        if(msgParts[1]=="1"){
          target_angle = value;
          current_angle = target_angle;
          //EEPROM.put( 10, target_angle );
          servo.write(target_angle);
          
        } else if(msgParts[1]=="2"){
          target2_angle = value;
          current2_angle = target2_angle;
          //EEPROM.put( 30, target2_angle );
          servo2.write(target2_angle);
        }
      
        Serial.println("Ok, applied angle " + String(value) + " to servo " + String(msgParts[1]));
      } else {
        Serial.println("OOF, invalid angle " + msgParts[1]);
      }
      Serial.println("after");
      return;

    } else if(msgParts[0]=="SENSORMAX"){ // SENSORMAX <integer max value>
      value = msgParts[1].toInt();
      
      if(!isnan(value)){
        sensorTooMuch = value;
        EEPROM.put( 20, sensorTooMuch );
        Serial.println("Ok, applied sensor max " + String(sensorTooMuch));
      } else {
        Serial.println("OOF, invalid sensor max value " + msgParts[1]);
      }
      return;
      
    } else if(msgParts[0]=="READING"){
      // DEBUGGING: display the current going through the wire
      String toPrint = "Currents: ";
      for(int i=0; i<servos; i++){
        toPrint += String(readings[i]) + " ";
      }
      Serial.println(toPrint);
      return;
    }
  
    Serial.println("OOF, unknown command " + msg);
  }
*/

  // Impedance Control
  /*
  int previous_angle = current_angle;
  if(readings[5]>sensorTooMuch){
    current_angle -= Step;
    if(current_angle<0) current_angle = 0;
  } else {
    current_angle += Step;
    if(current_angle>target_angle) current_angle = target_angle;
  }
  if(previous_angle!=current_angle)
    servo.write(current_angle);
  */
}




const int arrSize = 10;
String* getValues(String &msg){

    String* arr = new String[arrSize]; // according to the largest string array we'll need (ex. ANGLES 90 90 90 90 90)
    //for(int v=0; v<arrSize; v++){
    //    arr[v] = "";
    //}
    
    // if string is empty return
    if(msg=="") return arr;

    unsigned int j = 0;
    for (unsigned int i = 0; i < msg.length(); i++) {
        if(msg[i]==' ') j += 1;
        else arr[j] += msg[i];
    }
    return arr;
}
