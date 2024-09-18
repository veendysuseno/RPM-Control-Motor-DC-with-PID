// Driver-Test-MotorDC

#define POT_PIN A0
#define enA 9
#define in2 8
#define in1 7

void setup() {
  Serial.begin(9600);
  pinMode(POT_PIN, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  int analogValue = analogRead(POT_PIN);
  int analogMap = map(analogValue, 0, 1023, 0, 255);
  digitalWrite(in1, HIGH); // control motor direction (clockwise)
  digitalWrite(in2, LOW);  // control motor direction (clockwise)
  analogWrite(enA, analogMap); // control motor speed
  Serial.print("POT :");
  Serial.println(analogMap);
  delay(3000); // 3-second delay before the next update
}