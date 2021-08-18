#include <PMserial.h>
#include <SoftwareSerial.h>

SerialPM pmSensor1(PMSA003, 10, 11);
SerialPM pmSensor2(PMSA003, 8, 9);

void setup() {
  pmSensor1.init();
  pmSensor2.init();
}

void loop() {
  pmSensor1.read();
  pmSensor2.read();

  printToSerial(pmSensor1);
  printToSerial(pmSensor2);
}

void printToSerial(SerialPM pmSensor) {
  Serial.print(F("PM1.0 "));
  Serial.print(pmSensor.pm01);
  Serial.print(F(", "));
  // for particles less than 2.5ug/m3 ?
  Serial.print(F("PM2.5 "));
  Serial.print(pmSensor.pm25);
  Serial.print(F(", "));
  // for particles less than 10
  Serial.print(F("PM10 "));
  Serial.print(pmSensor.pm10);
  Serial.println(F(" [ug/m3]"));
}
