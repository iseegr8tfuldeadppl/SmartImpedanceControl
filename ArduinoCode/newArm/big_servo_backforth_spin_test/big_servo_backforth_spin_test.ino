#include <Servo.h>

Servo motor1, motor2;
long angle1 = 90, angle2 = 90;

void setup() {
  motor1.attach(5);
  motor2.attach(6);
  Serial.begin(9600);
  Serial.println("Ok,Started with angles " + String(angle1) + " " + String(angle2));
  motor1.write(angle1);
  motor2.write(angle2);
}

void loop() {
  if(Serial.available()){
    String msg = Serial.readStringUntil('\n');
    String* portions = getValues(msg);

    if(portions[0] == "1"){
      angle1 = portions[1].toInt();
      motor1.write(angle1);
      Serial.println("Ok, updated angles into " + String(angle1) + " " + String(angle2));
    } else if(portions[0] == "2"){
      angle2 = portions[1].toInt();
      motor2.write(angle2);
      Serial.println("Ok, updated angles into " + String(angle1) + " " + String(angle2));
    }
  }
  /*
  for(int i=1000; i<=2000; i++){
    motor1.writeMicroseconds(i);
    delay(50);
  }
  for(int i=2000; i>=1000; i--){
    motor1.writeMicroseconds(i);
    delay(50);
  }
  */
}




const int arrSize = 10;
String* getValues(String &msg){

    String* arr = new String[arrSize]; // according to the largest string array we'll need (ex. ANGLES 90 90 90 90 90)
    //for(int v=0; v<arrSize; v++){
    //    arr[v] = "";
    //}
    
    // if string is empty return
    if(msg==""){
        Serial.println("OOF, EMPTY");
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
