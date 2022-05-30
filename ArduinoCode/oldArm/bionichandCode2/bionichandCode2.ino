#include <Servo.h>
#include <EEPROM.h>

Servo servo, servo2;
long value;
long target_angle = 90;
long current_angle = 90;
int sensorTooMuch = 1;

long target2_angle = 90;
long current2_angle = 90;

void setup() {

  // DEBUGGING: serial communication
  Serial.begin(9600);
  while(!Serial){}

  // prepare hardware
  servo.attach(9);
  servo2.attach(6);

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
}

int Step = 1;
void loop() {

  // force control
  /*
  int sensor = analogRead(A7);
  if(sensor>=sensorTooMuch){
    current_angle -= Step;
    if(current_angle<0) current_angle = 0;
  } else {
    current_angle += Step;
    if(current_angle>target_angle) current_angle = target_angle;
  }

  
  sensor = analogRead(A6);
  if(sensor>=sensorTooMuch){
    current2_angle -= Step;
    if(current2_angle<0) current2_angle = 0;
  } else {
    current2_angle += Step;
    if(current2_angle>target2_angle) current2_angle = target2_angle;
  }

  // update servo
  servo.write(current_angle);
  servo2.write(current2_angle);
  */
  

  Serial.println("Current: " + String(analogRead(A5)));
  // Communication: reply to commands from user
  if(Serial.available()){
    String msg = Serial.readStringUntil('\n');
    String* msgParts = getValues(msg);
  
    if(msgParts[0]=="ANGLE"){ // ANGLE <servo number> <integer angle value>
      value = msgParts[2].toInt();
    
      if(!isnan(value)){
        
        if(msgParts[1]=="1"){
          target_angle = value;
          EEPROM.put( 10, target_angle );
          servo.write(target_angle);
        } else if(msgParts[1]=="2"){
          target2_angle = value;
          EEPROM.put( 30, target2_angle );
          servo2.write(target2_angle);
        }
      
        Serial.println("Ok, applied angle " + String(value) + " to servo + " + String(msgParts[1]));
      } else {
        Serial.println("OOF, invalid angle " + msgParts[1]);
      }
        
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
      Serial.println("Current: " + String(analogRead(A7)) + " " + String(analogRead(A6)) );
      return;
    }
  
    Serial.println("OOF, unknown command " + msg);
  }
}



const int arrSize = 10;
String* getValues(String &msg){

    String* arr = new String[arrSize]; // according to the largest string array we'll need (ex. ANGLES 90 90 90 90 90)
    //for(int v=0; v<arrSize; v++){
    //    arr[v] = "";
    //}
    
    // if string is empty return
    if(msg==""){
        //Serial.println("OOF, EMPTY");
        return arr;
    }

    unsigned int j = 0;
    for (unsigned int i = 0; i < msg.length(); i++) {
        if(msg[i]==' ')
            j += 1;
        else
            arr[j] += msg[i];
    }

    return arr;
}
