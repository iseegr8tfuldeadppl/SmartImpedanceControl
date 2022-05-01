#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(3, 2); // TX RX

void setup() {
  Bluetooth.begin(9600);
}

void loop() {
    // Read from Arduino Serial Monitor and send it to the HC-05
  //if (Bluetooth.available()){
  //  String yes = Bluetooth.readStringUntil('\n');
  //  Bluetooth.println(yes);
  //}
  Bluetooth.println(analogRead(A2));
  delay(300);
}
