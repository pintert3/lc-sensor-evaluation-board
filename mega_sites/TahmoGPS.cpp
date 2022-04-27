#include "TahmoGPS.h"

String getGPSinfo(TinyGPSPlus gps) {
  String output;
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated()) {
      output += String(gps.location.lat(), 6);
      output += ',';
      output += String(gps.location.lng(), 6);
    }
  }
  return output;
}
