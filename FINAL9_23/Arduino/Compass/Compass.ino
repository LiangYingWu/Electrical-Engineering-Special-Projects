
#include <QMC5883LCompass.h>

QMC5883LCompass compass;
int original_x = 0;
int original_y = 0;
int original_z = 0;
void setup() {
  Serial.begin(9600);
 
  
  // 初始化 QMC5883L
  compass.init();
  Serial.println("Compass Test");
  compass.read();
  original_x = compass.getX();
  original_y = compass.getY();
  original_z = compass.getZ();
  Serial.print(original_x);
  Serial.print(original_y);
  Serial.print(original_z);
}

void loop() {
  // 讀取磁場 X/Y/Z
  compass.read();
  int Currently_x = compass.getX();
  int Currently_y = compass.getY();
  int Currently_z = compass.getZ();
  int x = Currently_x-original_x ;
  int y = Currently_y-original_y ;
  int z = Currently_z-original_z ;
  

  // 計算 Heading（方位角）
  float heading = atan2((float)Currently_y, (float)Currently_x);  // 弧度
  if (heading < 0) heading += 2 * PI;        // 轉成 0~2PI
  if (heading > 2 * PI) heading -= 2 * PI;

  float headingDegrees = heading * 180.0 / PI;  // 弧度轉角度
  Serial.print(" | Heading: "); Serial.print(headingDegrees);
  Serial.println("°");

  delay(500);
}
