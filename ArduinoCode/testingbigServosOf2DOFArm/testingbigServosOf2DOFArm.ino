#include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(9);
}

long angle = 90;
int previous_force = 0;
int previous_speed = 0;
long Ki = 0; // inertia
long Kd = 0; // damping
long Ks = 1; // stiffness
int period = 100; // in ms

// temp variables
long displacement, Speed, acceleration;
long start_of_period;
void loop() {

  start_of_period = millis();

  float sensor2 = analogRead(A6);
  float sensor1 = analogRead(A7);
  float forceFloat = ( sensor1 - sensor2 ) / 1023 * 60;
  int force =  forceFloat;
  long Speed = force - previous_force;
  long acceleration = Speed - previous_speed;
  long wantedSteps = Ks * force + Kd * Speed + Ki * acceleration;

  angle = angle + wantedSteps;
  
  if(angle>180) angle = 180;
  if(angle<0) angle = 0;

  servo.write(angle);
  previous_force = force;
  previous_speed = Speed;
  
  Serial.println("Total force " + String(force) + " " + String(analogRead(A7)) + " " + String(analogRead(A6)) );
  /*
  for(int i=0; i<220; i++){
    servo.write(i);
    delay(10);
  }
  
  for(int i=219; i>=0; i--){
    servo.write(i);
    delay(10);
  }
  */

  // to ensure we have the same period for all processings
  delay(period - (millis() - start_of_period) );
}
