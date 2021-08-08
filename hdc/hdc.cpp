#include "hdc.h"
uint8_t HDC1080::begin()
{
  delay(20);
  Wire.begin();
  //Write to configuration register
  Wire.beginTransmission(HDC1080_ADDR); // I2C Adress of HDC1080
  Wire.write(0x02);           // Point to configuration register on 0x02
  Wire.write(0x90);           // Configuration 1 0 0 1 0 0 0 0 0x00
  Wire.write(0x00);           //

  return Wire.endTransmission();
}

double HDC1080::getTemperature()
{
  uint8_t Byte[4];
  uint16_t temp;

  Wire.beginTransmission(HDC1080_ADDR);
  Wire.write(0x00); //Point to temperature register
  Wire.endTransmission();

  delay(20); //conversion time

  Wire.requestFrom(HDC1080_ADDR, 4); //Request 4 bytes of data

  //store 4 bytes of data in the array
  if (4 <= Wire.available())
  {
    Byte[0] = Wire.read();
    Byte[1] = Wire.read();
    Byte[3] = Wire.read();
    Byte[4] = Wire.read();

    temp = (((unsigned int)Byte[0] << 8 | Byte[1]));
    return (double)(temp) / (65536) * 165 - 40;
  }
  else
    return 0;
}

double HDC1080::getHumidity()
{
  uint8_t Byte[4];
  uint16_t humi;

  Wire.beginTransmission(HDC1080_ADDR);
  Wire.write(0x00); //Point to temperature register
  Wire.endTransmission();

  delay(20); //conversion time

  Wire.requestFrom(HDC1080_ADDR, 4); //Request 4 bytes of data

  //store 4 bytes of data in the array
  if (4 <= Wire.available())
  {
    Byte[0] = Wire.read();
    Byte[1] = Wire.read();
    Byte[3] = Wire.read();
    Byte[4] = Wire.read();

    humi = (((unsigned int)Byte[3] << 8 | Byte[4]));
    return (double)(humi) / (65536) * 100;
  }
}
