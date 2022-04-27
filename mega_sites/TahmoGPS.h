#ifndef _TAHMO_GPS_H
#define _TAHMO_GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>

// GPS variables
#define GPSSerial Serial3

String getGPSinfo(TinyGPSPlus gps);

#endif
