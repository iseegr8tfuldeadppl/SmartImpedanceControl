#include <Servo.h>


Servo servo, servo2, servo3, servo4, servo5;

long real_servo_position;
long desired_servo_position;
long previous_desired_servo_position;


long sensor1=0, sensor2=0;

long zero = 572;

long samples_per_second = 10;
long previous_current_reading;


long counter_current_sensor_1 = 0;
long counter_current_sensor_2 = 0;

long total_current_sensor_1 = 0;
long total_current_sensor_2 = 0;


void setup() {
  Serial.begin(9600);
  
  servo.attach(5);
  servo2.attach(6);
  servo3.attach(9);
  servo4.attach(10);
  servo5.attach(11);

  pinMode(2, INPUT);

  readPotentiometer();
  updateServos();

  delay(1000);
  
  previous_current_reading = millis();

}


void readPotentiometer(){
  desired_servo_position = map(analogRead(A5), 0, 1023, 1, 179);
}

void updateServos(){
  servo.write(real_servo_position);
  servo2.write(180-real_servo_position);
  servo3.write(180-real_servo_position);
  servo4.write(real_servo_position);
  servo5.write(180-real_servo_position);
  previous_desired_servo_position = desired_servo_position;
}


void loop() {

  // adjust the zero threshhold nothing big
  /*if(digitalRead(2)==HIGH){
    int minimum = analogRead(A7);
    int minimum2 = analogRead(A6);
    if(minimum < minimum2)
      zero = minimum;
    else
      zero = minimum2;
  }*/

  // update current-consumption readings
  if(millis() - previous_current_reading >= 1000/samples_per_second){
    previous_current_reading = millis();
    sensor1 = total_current_sensor_1;
    total_current_sensor_1 = 0;
    counter_current_sensor_1 = 0;

    //Serial.println("current: " + String(zero-sensor1));

      /*
      // impedance control
      if(zero-sensor1>50){
        real_servo_position += 3;
        if(real_servo_position > 179)
          real_servo_position = 179;
        else {
          updateServos();
          delay(500);
          previous_current_reading = millis();
        }
      } else{
        real_servo_position -= 3;
        if(real_servo_position < desired_servo_position) // because servos closing is proportional to potentiometer value reducing
          real_servo_position = desired_servo_position;
        else {
          updateServos();
          delay(500);
          previous_current_reading = millis();
        }
      }
      */
  
  } else {
    // keep averaging sensor values until we reach the time for next sample
    total_current_sensor_1 = ( analogRead(A7) + counter_current_sensor_1 * total_current_sensor_1) / (counter_current_sensor_1 + 1);
    counter_current_sensor_1 += 1;
  }


  // update servos according to potentiometer
  readPotentiometer();
  if(abs(desired_servo_position-previous_desired_servo_position) >= 4){
    real_servo_position = desired_servo_position;
    updateServos();
    previous_current_reading = millis();
  }
  
}
