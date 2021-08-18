#pragma

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Wire.h"
#define HDC1080_ADDR 0x40
class HDC1080
{ //Ausgabe in Fahrenheit
public:
	uint8_t begin(void);
	double getTemperature(void);
	double getHumidity(void);
};
