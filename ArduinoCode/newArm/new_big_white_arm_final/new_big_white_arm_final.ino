#include <Servo.h>

// sensor variables
const int sensor_count = 4;
// back is right
// forward is back
int sensor_analog_pins[sensor_count] = {A3, A5, A1, A7};

double max_sensor_vals_report_hz = 5; // frequency of sensor reports to python per second
long max_sensor_vals_report_period = 1000 / max_sensor_vals_report_hz; // in milliseconds
long previous_report_time; // in milliseconds
String report = "";

// servo motors variables
Servo motor1, motor2;
long angle1 = 90, angle2 = 90;
int Speed = 5; // how many steps to take to reach desired position
float angle1_stepp, angle2_stepp; // temp variables

// serial communication variables
const int arrSize = 10;
String* parts_of_msg = new String[arrSize]; // according to the largest string array we'll need (ex. ANGLES 90 90 90 90 90)

void setup() {
  Serial.begin(9600);
  motor1.attach(6);
  motor2.attach(5);
  
  motor1.write(angle1);
  motor2.write(angle2);
  Serial.println("Ok, started with angles:" + String(angle1) + " " + String(angle2));
}

void loop() {

  // read sensor values, report if appropriate time
  if(millis() - previous_report_time > max_sensor_vals_report_period){
    previous_report_time = millis();
    report = "Ok, sensors:";
    for(int i=0; i<sensor_count; i++)
      report +=  " " + String(1023 - analogRead(sensor_analog_pins[i]));
    Serial.println(report);
  }

  // receive commands from python
  if(Serial.available()){
    String msg = Serial.readStringUntil('\n');
    
    getValues(msg);

    if(parts_of_msg[0] == "ANGLES"){ // ANGLES <angle1> <angle2>
      long temp_angle1 = parts_of_msg[1].toInt();
      long temp_angle2 = parts_of_msg[2].toInt();
      go_to_coordinates(temp_angle1, temp_angle2, Speed); // this is to go to a certain coordinate synchronously with tuned speed
      
      Serial.println("Ok, updating angles into " + String(angle1) + " " + String(angle2));
      Serial.flush();
      return;
    } else if(msg == "REPORT"){ // REPORT
      Serial.println("Ok, angles are " + String(angle1, DEC) + " " + String(angle2));
      Serial.flush();
      return;
    }

    Serial.println("OOF, unknown command " + String(msg));
    Serial.flush();
  }
}




void go_to_coordinates(int m1, int m2, int Speed){

  int another_speed = 10;
  angle1_stepp = (m1 - angle1)/another_speed;
  angle2_stepp = (m2 - angle2)/another_speed;

  for(int i= 0; i < another_speed; i++){
    angle1 += angle1_stepp;
    motor1.write(angle1);
    
    angle2 += angle2_stepp;
    motor2.write(angle2);
    
    //delay(30); // 30 or 60 were both good
  }

  angle1 = m1;
  angle2 = m2;
  
  motor1.write(angle1);
  motor2.write(angle2);

}


unsigned int j = 0;
void getValues(String msg){
    j = 0;
    parts_of_msg[0] = "";
    for (unsigned int i = 0; i < msg.length(); i++) {
        if(j>=arrSize)
          break;
        if(msg[i]==' '){ j += 1; parts_of_msg[j] = ""; }
        else parts_of_msg[j] += msg[i];
    }
}
