#include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(9600);

  servo.attach(9);
  servo.write(180);
}

int mode = 1;
void loop() {
  if(mode==1){
    for(int i=0; i<180; i++){
      servo.write(i);
      delay(4 );
    }
    for(int i=180; i>0; i--){
      servo.write(i);
      delay(4);
    }
  } else if(mode==0){
  }

}
