void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Current: " + String(analogRead(A7)));
}
