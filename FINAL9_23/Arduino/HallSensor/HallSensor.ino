float wc = 80.2; // 80.2 cm 

// 霍爾感測器狀態
int hallACounter = 0;
int hallBCounter = 0;
int hallCCounter = 0;

int hallALast = 0;
int hallBLast = 0;
int hallCLast = 0;

// 三條線的上次觸發時間
unsigned long lastMicrosA = 0;
unsigned long lastMicrosB = 0;
unsigned long lastMicrosC = 0;

// 三條線的轉速
float rpsA = 0;
float rpsB = 0;
float rpsC = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorA = analogRead(A0);
  int sensorB = analogRead(A1);
  int sensorC = analogRead(A2);
  float gas = analogRead(A3) * 5.0 / 1024.0;

  // A 線霍爾偵測
  if ((hallALast == 0) && (sensorA > 512)) {
    hallALast = 1;
    hallACounter++;
    rpsA = calculateRPS(&lastMicrosA);
  } else if ((hallALast == 1) && (sensorA < 512)) {
    hallALast = 0;
  }

  // B 線霍爾偵測
  if ((hallBLast == 0) && (sensorB > 512)) {
    hallBLast = 1;
    hallBCounter++;
    rpsB = calculateRPS(&lastMicrosB);
  } else if ((hallBLast == 1) && (sensorB < 512)) {
    hallBLast = 0;
  }

  // C 線霍爾偵測
  if ((hallCLast == 0) && (sensorC > 512)) {
    hallCLast = 1;
    hallCCounter++;
    rpsC = calculateRPS(&lastMicrosC);
  } else if ((hallCLast == 1) && (sensorC < 512)) {
    hallCLast = 0;
  }

  // 每 100ms 印出一次速度
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    unsigned long now = micros();

    // 如果超過 300ms 沒訊號，就把 rps 歸零
    if (now - lastMicrosA > 250000) rpsA = 0;
    if (now - lastMicrosB > 250000) rpsB = 0;
    if (now - lastMicrosC > 250000) rpsC = 0;

    // Serial.print("Gas: ");
    // Serial.print(gas);
    // Serial.print(" | RPS A: ");
    // Serial.print(rpsA);
    // Serial.print(" | RPS B: ");
    // Serial.print(rpsB);
    // Serial.print(" | RPS C: ");
    // Serial.print(rpsC);
    // Serial.print(" | Average RPS: ");
    // float averageRps = (rpsA + rpsB + rpsC) / 3;
    // Serial.print(averageRps);
    // Serial.print(" | Average Speed: ");
    // float averageSpeed = averageRps * wc / 16 * 7 / 100;
    // Serial.print(averageSpeed);
    // Serial.println(" (m/s)");
    
    float averageRps = (rpsA + rpsB + rpsC) / 3;
    float averageSpeed = averageRps * wc / 16 * 7;
    Serial.print("gps,");
    Serial.print("1,");
    Serial.print("2,");
    Serial.print("campass,");
    Serial.print("4,");
    Serial.print("hall,");
    Serial.print(averageSpeed);
    Serial.print(",");
    Serial.print(rpsA);
    Serial.print(",");
    Serial.print(rpsB);
    Serial.print(",");
    Serial.print(rpsC);
    Serial.print(",");
    Serial.print("SR-04,");
    Serial.print("11,");
    Serial.print("12,");
    Serial.print("13,");
    Serial.print("14,");
    Serial.println("15");
    lastPrint = millis();
  }
}

float calculateRPS(unsigned long* lastMicros) {
  unsigned long currentMicros = micros();
  unsigned long deltaMicros = currentMicros - *lastMicros;
  *lastMicros = currentMicros;

  if (deltaMicros == 0) return 0;

  // 一圈有 30 個脈波
  float revPerMicros = 1.0 / (deltaMicros * 30);
  float rps = revPerMicros * 1000000.0;

  return rps;
}

float calculateRPM(unsigned long* lastMicros) {
  unsigned long currentMicros = micros();
  unsigned long deltaMicros = currentMicros - *lastMicros;
  *lastMicros = currentMicros;

  if (deltaMicros == 0) return 0;

  // 一圈有 30 個脈波
  float revPerMicros = 1.0 / (deltaMicros * 30);
  float rpm = revPerMicros * 60.0 * 1000000.0;

  return rpm;
}
