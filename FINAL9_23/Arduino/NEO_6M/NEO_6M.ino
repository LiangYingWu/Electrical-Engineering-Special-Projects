#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

SoftwareSerial NEO_6M(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  NEO_6M.begin(GPSBaud);
}

void loop() {
  while (NEO_6M.available() > 0)
    if (gps.encode(NEO_6M.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

}
