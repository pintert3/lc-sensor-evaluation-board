#include <PMserial.h>

SerialPM pmsa = SerialPM(PMSA003, Serial1);
void setup() {
  pmsa.init();
  Serial.begin(9600);
}

void loop() {
  delay(2000);
  pmsa.read();
  Serial.print("PM01: ");
  Serial.println(pmsa.pm01);
  Serial.print("PM25: ");
  Serial.println(pmsa.pm25);
  Serial.print("PM10: ");
  Serial.println(pmsa.pm10);
  pmsa.sleep();
  delay(5000);
  pmsa.wake();
}
