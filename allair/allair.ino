#include "SdsDustSensor.h"
#include <PMserial.h>
#include "SoftwareSerial.h"
SoftwareSerial soft[3]={SoftwareSerial(10,11),SoftwareSerial(12,13),SoftwareSerial(50,51)};
SdsDustSensor sds[3]={SdsDustSensor(soft[0]),SdsDustSensor(soft[1]),SdsDustSensor(soft[2])}; //passing Softwareserial& as parameter
SerialPM pmsa_array[3]={SerialPM(PMSA003, Serial1),SerialPM(PMSA003, Serial2),SerialPM(PMSA003, Serial3)};// passing HardwareSerial& as parameter
void setup() {
  
  Serial.begin(9600);
  //initialize the sensors
  for(int i=0;i<3;i++){
    pmsa_array[i].init();
    sds[i].begin(); // this line will begin soft-serial with given baud rate (9600 by default)
    sds[i].setQueryReportingMode(); // set reporting mode
 
}}

void loop() {
  // wake the sds up
  for(int i=0;i<3;i++){
    soft[i].listen();
    sds[i].wakeup();
    }
    delay(40000);
  for(int i=0;i<3;i++){
    soft[i].listen();
    PmResult pm = sds[i].queryPm();
  if (pm.isOk()) {
    Serial.print("PM2.5 = ");
    Serial.print(pm.pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm.pm10);
  } else {
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }
 
     WorkingStateResult state = sds[i].sleep();
  if (state.isWorking()) {
    Serial.println("Problem with sleeping the sensor.");
  } else {
    Serial.println("Sensor is sleeping");
    
  }
   
  delay(1000);
  
  pmsa_array[i].read();
    if (pmsa_array[i]) {
      // print results
      // instead of printing, should be sending the data to
      // a responsible sd card location in the format required.

      // for particles less than 1.0ug/m3
      Serial.print(F("PM1.0 "));
      Serial.print(pmsa_array[i].pm01);
      Serial.print(F(", "));
      // for particles less than 2.5ug/m3 ?
      Serial.print(F("PM2.5 "));
      Serial.print(pmsa_array[i].pm25);
      Serial.print(F(", "));
      // for particles less than 10
      Serial.print(F("PM10 "));
      Serial.print(pmsa_array[i].pm10);
      Serial.println(F(" [ug/m3]"));
    /*
      if (pmsa_array[i].has_number_concentration())
      {
        Serial.print(F("N0.3 "));
        Serial.print(pmsa_array[i].n0p3);
        Serial.print(F(", "));
        Serial.print(F("N0.5 "));
        Serial.print(pmsa_array[i].n0p5);
        Serial.print(F(", "));
        Serial.print(F("N1.0 "));
        Serial.print(pmsa_array[i].n1p0);
        Serial.print(F(", "));
        Serial.print(F("N2.5 "));
        Serial.print(pmsa_array[i].n2p5);
        Serial.print(F(", "));
        Serial.print(F("N5.0 "));
        Serial.print(pmsa_array[i].n5p0);
        Serial.print(F(", "));
        Serial.print(F("N10 "));
        Serial.print(pmsa_array[i].n10p0);
        Serial.println(F(" [#/100cc]"));
      }
  
      if (pmsa_array[i].has_temperature_humidity() || pmsa_array[i].has_formaldehyde())
      {
        Serial.print(pmsa_array[i].temp, 1);
        Serial.print(F(" °C"));
        Serial.print(F(", "));
        Serial.print(pmsa_array[i].rhum, 1);
        Serial.print(F(" %rh"));
        Serial.print(F(", "));
        Serial.print(pmsa_array[i].hcho, 2);
        Serial.println(F(" mg/m3 HCHO"));
      }*/
    } else {
      switch (pmsa_array[i].status)
      {
        case pmsa_array[i].OK: // should never come here
          break;     // included to compile without warnings
        case pmsa_array[i].ERROR_TIMEOUT:
          Serial.println(F(PMS_ERROR_TIMEOUT));
          break;
        case pmsa_array[i].ERROR_MSG_UNKNOWN:
          Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
          break;
        case pmsa_array[i].ERROR_MSG_HEADER:
          Serial.println(F(PMS_ERROR_MSG_HEADER));
          break;
        case pmsa_array[i].ERROR_MSG_BODY:
          Serial.println(F(PMS_ERROR_MSG_BODY));
          break;
        case pmsa_array[i].ERROR_MSG_START:
          Serial.println(F(PMS_ERROR_MSG_START));
          break;
        case pmsa_array[i].ERROR_MSG_LENGTH:
          Serial.println(F(PMS_ERROR_MSG_LENGTH));
          break;
        case pmsa_array[i].ERROR_MSG_CKSUM:
          Serial.println(F(PMS_ERROR_MSG_CKSUM));
          break;
        case pmsa_array[i].ERROR_PMS_TYPE:
          Serial.println(F(PMS_ERROR_PMS_TYPE));
          break;
      }
  
    }
  }
  //sleeping the sds
  
  
  
  delay(30000);//wait a minute
  
 
}
