#include <TinyGPS++.h>

#define GPS_SERIAL Serial1

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);
}

void loop() {
  while (GPS_SERIAL.available()) {
    if (gps.location.isUpdated()) {
      Serial.print("Latitude=");
      Serial.print(gps.location.lat(), 6);
      Serial.print("Longitude=");
      Serial.print(gps.location.lng(), 6);
    }
  }
}
