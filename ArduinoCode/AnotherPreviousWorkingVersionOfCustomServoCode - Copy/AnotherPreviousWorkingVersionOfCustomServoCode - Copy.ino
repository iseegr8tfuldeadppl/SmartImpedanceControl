void setup() {
  Serial.begin(9600);
  Serial.println("Started");

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(7, INPUT);
  pinMode(8, INPUT);

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  analogWrite(6, 0);

}

// TODO:
// implement onlysmall speeds when we're near the wanted area

// GOOD SETTINGS:
//Speed=150, okayZone=50 or maybe more
//Speed=200, okayZone=90 or maybe less
//HighSpeed=150, lowSpeed=66, cbnZone=80, slowZone=20

int Speed; // lowest speed 45

//#define safety 24
int desiredAngle = 512;

#define cbnZone 60 // 60
#define slowZone 30 // 30
#define lowSpeed 66 // 66
#define highSpeed 80 // 100

int enterTime;
int correctionTime = 30; // in ms

int debugStep = 30; // 50
boolean Continue = false;
boolean buttonUp = true;
void loop() {
  /*potentiometer = analogRead(A7);
  Serial.println("Current: " + String(potentiometer) + " Desired " + String(desiredAngle) + " Currents: " + String(analogRead(A6)));

  if(digitalRead(7)==HIGH){
    Speed = highSpeed;
    analogWrite(6, Speed);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
  } else if(digitalRead(8)==HIGH){
    Speed = highSpeed;
    analogWrite(6, Speed);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
  } else {
    Speed = highSpeed;
    analogWrite(6, Speed);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
  }*/

  if(Continue){
    Continue = false;

    // continious back and forth
    desiredAngle += debugStep;
    if(desiredAngle>=900){
      desiredAngle = 900;
      debugStep = -debugStep;
    } else if(desiredAngle<=100){
      desiredAngle = 100;
      debugStep = -debugStep;
    }

  }

    //Serial.println("Angle: " + String(analogRead(A7)) + " Desired: " + String(desiredAngle) + " Currents: " + String(analogRead(A6)));
    if(abs(analogRead(A7) - desiredAngle) > slowZone/2){
  
      if(analogRead(A7) - desiredAngle > 0){
        //Serial.println("Clockwise"); // turns down the potentiometer value with white in ground, purple 5v, gray A7
        Speed = highSpeed;
        analogWrite(6, Speed);
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);

        Serial.println("positive delta");
        while(analogRead(A7) - desiredAngle > cbnZone/2){}
        Speed = lowSpeed;
        analogWrite(6, Speed);
        enterTime = millis();
        while(millis()-enterTime<correctionTime && analogRead(A7) - desiredAngle > slowZone/2){}
        digitalWrite(4, LOW);
        Continue = true;
        
      } else if(analogRead(A7) - desiredAngle < 0){
        Speed = highSpeed;
        analogWrite(6, Speed);
        
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
      
        Serial.println("negative delta");
        while(desiredAngle - analogRead(A7) < -cbnZone/2){}
        Speed = lowSpeed;
        analogWrite(6, Speed);
        enterTime = millis();
        while(millis()-enterTime<correctionTime && desiredAngle - analogRead(A7) < -slowZone/2){}
        digitalWrite(5, LOW);
        Continue = true;
        
      }
      
    } else {
      Continue = true;
    }

}