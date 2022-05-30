#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial Bluetooth(3, 2); // TX RX
Servo servo;

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600); //38400

  servo.attach(9);
  servo.write(90);
}

void loop() {
    // Read from Arduino Serial Monitor and send it to the HC-05
  /*if (Serial.available()){
    String yes = Serial.readStringUntil('\n');
    Bluetooth.println(yes);
  }*/

  
    
  if (Bluetooth.available()){
    String yes = Bluetooth.readStringUntil('\n');
    Serial.println(yes);
  }
}
