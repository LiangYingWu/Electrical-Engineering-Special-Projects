int GAS_PUL_PIN = 3;  // 油門脈衝腳
int GAS_DIR_PIN = 4; // 油門方向腳
int ELECTRIC_FRONT_PIN=7; // 電動缸前進
int ELECTRIC_BACKWARD_PIN=8;  // 電動缸後退


char input_buffer[50];   // 接收緩衝區
int input_idx = 0;

int step_memory;
int step_motor_delta_step;
int electric_cylinder_delta;
int electric_cylinder_memory;
int electric_count;
bool fir = false;

void parseData(char *data);
void pulseOutput(int pulPin, int dir, int count);

void setup() {
  Serial.begin(9600);

  pinMode(GAS_PUL_PIN, OUTPUT); //油門pin宣告
  pinMode(GAS_DIR_PIN, OUTPUT);

  digitalWrite(GAS_PUL_PIN, LOW); //油門脈衝
  digitalWrite(GAS_DIR_PIN, LOW); //油門方向
  step_memory = 0;
  step_motor_delta_step = 0;
  electric_count=0;
  electric_cylinder_delta = 0;
  electric_cylinder_memory = 0;
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      input_buffer[input_idx] = '\0';
      parseData(input_buffer);
      input_idx = 0;
    } 
    else {
      input_buffer[input_idx++] = c;
      if (input_idx >= sizeof(input_buffer)) input_idx = 0;
    }
    
    if (step_motor_delta_step > 0){
      if (fir) {
        pulseOutput(GAS_PUL_PIN, 1, step_motor_delta_step);
      }
      else {
        pulseOutput(GAS_PUL_PIN, 1, 140);
        fir = true;
      }
    }
    else if (step_motor_delta_step < 0) {
      pulseOutput(GAS_PUL_PIN, -1, (step_motor_delta_step * -1));
    }
  }
    step_motor_delta_step = 0;

    electric_count=electric_cylinder_memory-electric_cylinder_delta;

    
  if (electric_count < 0)
  {
    electricOutput(ELECTRIC_FRONT_PIN,electric_count);
  }  
  else if (electric_cylinder_delta > 0)
  {
    electricOutput(ELECTRIC_BACKWARD_PIN,electric_count);
  }
    electric_count = 0;
  if(electric_count == 0 && step_motor_delta_step == 0)
  {
    Serial.println("DONE");
  }
    
  
}

void parseData(char *data) {
  char *token;
  int count = 0;

  token = strtok(data, ",");
  while (token != NULL) {
    switch (count) {
      case 0:
//        strncpy(piStr, token, sizeof(piStr));
        break;
      case 1:
        step_motor_delta_step = atoi(token);
        break;
      case 2: 
//        strncpy(ecStr, token, sizeof(ecStr));
        break;
      case 3: 
        electric_cylinder_delta = atoi(token);
        break;
    }
    token = strtok(NULL, ",");
    count++;
  }
}

void pulseOutput(int pulPin, int dir, int count) {
  if(dir == 1 && pulPin == GAS_PUL_PIN){
    digitalWrite(GAS_DIR_PIN, HIGH); 
  } 
  else if(dir == -1 && pulPin == GAS_PUL_PIN) {
    digitalWrite(GAS_DIR_PIN, LOW);
  }

  for (int i = 0; i < count; i++) {
    step_memory += dir;
    
    if (dir == 1 && step_memory >= 200) {
      break;
    }
    else if (dir == -1 && step_memory <= 10) {
      break;
    }

    digitalWrite(pulPin, HIGH);
    delay(1);
    digitalWrite(pulPin, LOW);
    delay(1);
  }
}
void electricOutput(int pulPin, int count)
{
  int time ;
  time=count*50;
  digitalWrite(pulPin, HIGH);
  delay(time);
  digitalWrite(pulPin, LOW);
  delay(10);
  electric_cylinder_memory=electric_cylinder_delta;

}

