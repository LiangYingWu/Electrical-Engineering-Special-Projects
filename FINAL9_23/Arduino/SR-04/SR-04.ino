// Arduino 程式碼
const int NUM_SENSORS = 6;
const int trigPins[NUM_SENSORS] = {2, 3, 4, 5, 6, 7};
const int echoPins[NUM_SENSORS] = {8, 9, 10, 11, 12, 13};
const char* directions[6] = {"Left-Forward", "Front", "Right-Forward","Left", "Right", "Back"};
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout: 30ms
  long distance = duration * 0.034 / 2;
  return distance;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(trigPins[i], OUTPUT);
      pinMode(echoPins[i], INPUT);
    }
}

void loop() {
 
 char command = Serial.read();
 if(commend == 1)
 {
  bool detected = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    long d = readDistance(trigPins[i], echoPins[i]);
    if (d > 0 && d < 20) {
      Serial.print("DETECTED at ");
      Serial.println(directions[i]);
      detected = true;
    }
    delay(50); // 每個感測器之間稍微間隔
  }

  if (!detected) {
    Serial.println("ALL SAFE");
  }


 }
  
  delay(500); // 每秒掃描一次

}
