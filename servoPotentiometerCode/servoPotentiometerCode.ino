#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(9);
}

void loop() {
  int valeur_analog = analogRead(A5);
  int degree = map(valeur_analog, 0, 1023, 0, 180);
  servo.write(degree);
}
