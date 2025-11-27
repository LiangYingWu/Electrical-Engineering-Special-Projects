#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

// 校正變數
long xmin =  999999, xmax = -999999;
long ymin =  999999, ymax = -999999;
long zmin =  999999, zmax = -999999;

float x_offset, y_offset, z_offset;
float x_scale, y_scale, z_scale;

unsigned long startTime;
const unsigned long CALIB_TIME = 20000; // 校正時間 10 秒

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();

  Serial.println("HA5883 (QMC5883L) 校正開始，請慢慢旋轉模組 360°");
  startTime = millis();
}

void loop() {
  //  compass.init();
  // delay(10);
  compass.read();
  long x = compass.getX();
  long y = compass.getY();
  long z = compass.getZ();

  // 更新 min/max
  if (x < xmin) xmin = x;
  if (x > xmax) xmax = x;
  if (y < ymin) ymin = y;
  if (y > ymax) ymax = y;
  if (z < zmin) zmin = z;
  if (z > zmax) zmax = z;

  // 校正時間結束
  if (millis() - startTime >= CALIB_TIME) {
    // 計算 offset
    x_offset = (xmax + xmin) / 2.0;
    y_offset = (ymax + ymin) / 2.0;
    z_offset = (zmax + zmin) / 2.0;

    // 計算 scale
    float x_half = (xmax - xmin) / 2.0;
    float y_half = (ymax - ymin) / 2.0;
    float z_half = (zmax - zmin) / 2.0;
    float avg_half = (x_half + y_half + z_half) / 3.0;

    x_scale = avg_half / x_half;
    y_scale = avg_half / y_half;
    z_scale = avg_half / z_half;

    // 印出校正結果
    Serial.println("===== 校正完成 =====");
    Serial.print("X offset: "); Serial.println(x_offset);
    Serial.print("Y offset: "); Serial.println(y_offset);
    Serial.print("Z offset: "); Serial.println(z_offset);

    Serial.print("X scale: "); Serial.println(x_scale);
    Serial.print("Y scale: "); Serial.println(y_scale);
    Serial.print("Z scale: "); Serial.println(z_scale);
    Serial.println("====================");

    while (1); // 停在這裡，不再更新
  }

  delay(50); // 50ms 更新一次
}
