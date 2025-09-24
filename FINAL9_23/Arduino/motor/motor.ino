// 創建腳位
int GAS_PUL_PIN=3;  // 油門脈衝腳
int GAS_DIR_PIN=4; // 油門方向腳

int BREAK_PUL_PIN=5; // 煞車脈衝腳
int BREAK_DIR_PIN=6; // 煞車方向腳

int ELECTRIC_FRONT_PIN=7; // 電動缸前進
int ELECTRIC_BACKWARD_PIN=8;  // 電動缸後退

int LEFT_DETECT_PIN=9; //左側觸發開關
int MID_DETECT_PIN=10; //中間觸發開關
int RIGHT_DETECT_PIN=11; //右側觸發開關
void setup() {
  // 宣告腳位
  pinMode(GAS_PUL_PIN, OUTPUT); //油門pin宣告
  pinMode(GAS_DIR_PIN, OUTPUT);
  
  pinMode(BREAK_PUL_PIN, OUTPUT); //煞車pin宣告
  pinMode(BREAK_DIR_PIN, OUTPUT);
  
  pinMode(ELECTRIC_FRONT_PIN, OUTPUT);     //電缸pin宣告
  pinMode(ELECTRIC_BACKWARD_PIN, OUTPUT);  
  
  pinMode(RIGHT_DETECT_PIN, INPUT); //觸發開關pin宣告(input)
  pinMode(LEFT_DETECT_PIN, INPUT);
  pinMode(MID_DETECT_PIN, INPUT);

   //歸零輸出
  digitalWrite(GAS_PUL_PIN, LOW); //油門脈衝
  digitalWrite(GAS_DIR_PIN, LOW); //油門方向

  digitalWrite(BREAK_PUL_PIN, LOW); //煞車脈衝
  digitalWrite(BREAK_DIR_PIN, LOW); //煞車方向

  digitalWrite(ELECTRIC_FRONT_PIN, HIGH); //電缸前進
  digitalWrite(ELECTRIC_BACKWARD_PIN, HIGH); //電缸後退
  
  //龅率設定
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char command = Serial.read(); // 讀取一個字元命令
    int front = 1 ;
    int back = 2 ;
    switch (command) {
      case '1': // 電動缸正轉
        Serial.println(F("Turn Left"));
        moveActuator(ELECTRIC_FRONT_PIN,2000);
        Serial.println(F("DONE"));
        break;
      case '2': // 電動缸反轉
        Serial.println(F("Turn Right"));
        moveActuator(ELECTRIC_BACKWARD_PIN,2000);
        Serial.println(F("DONE"));
        break;
      case '3': // 油門馬達正轉
        Serial.println(F("Speed Up"));
        pulseOutput(GAS_PUL_PIN,front,50); // 正轉方向
        Serial.println(F("DONE"));
        break;
      case '4': // 油門馬達反轉
        Serial.println(F("Speed down"));
        pulseOutput(GAS_PUL_PIN,back,50); // 反轉方向
        Serial.println(F("DONE"));
        break;
      case '5': // 煞車馬達正轉
        Serial.println(F("Breaking"));
        pulseOutput(BREAK_PUL_PIN,front,500); // 正轉方向
        Serial.println(F("DONE"));
        break;
      case '6': // 煞車馬達反轉
        Serial.println(F("Breakloseing"));
        pulseOutput(BREAK_PUL_PIN,back,500); // 反轉方向
        Serial.println(F("DONE"));
        break;
      default:
        // 其他指令不執行
        Serial.println(F("DONE"));
        break;
    }
  }
}
void pulseOutput(int pulPin, int dir, int count)
{
  if(pulPin == GAS_PUL_PIN ){

    if(dir == 1){
      digitalWrite(GAS_DIR_PIN, HIGH);
    }
    else if(dir == 2){
      digitalWrite(GAS_DIR_PIN, LOW);
    }
    
  } 
  if(pulPin == BREAK_PUL_PIN ){

    if(dir == 1){
      digitalWrite(BREAK_DIR_PIN, HIGH);
    }
    else if(dir == 2){
       digitalWrite(BREAK_DIR_PIN, LOW);
    }
    
  }
  
  for (int i = 0; i < count; i++) {
    
    digitalWrite(pulPin, HIGH);
    delay(1);
    digitalWrite(pulPin, LOW);
    delay(1);
  }

}
void moveActuator(int dirPin,int time) 
{
  
  digitalWrite(ELECTRIC_FRONT_PIN, HIGH);
  digitalWrite(ELECTRIC_BACKWARD_PIN, HIGH);
  digitalWrite(dirPin, LOW);
  delay(time); // 動作時間
  digitalWrite(dirPin, HIGH);

}
