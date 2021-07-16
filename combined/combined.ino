#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include<SPI.h>
#include "SD.h"
#include "Adafruit_SHT31.h"
#include "ClosedCube_HDC1080.h"
#include "Adafruit_HTU21DF.h"
#include "SdsDustSensor.h"
#include <PMserial.h>
#include "SoftwareSerial.h"
#include <DS3231.h>

DS3231  rtc(SDA, SCL);
//sdcard
const int CSpin = 53;
File tempFile;
File humFile;
File airPartFile;
//tiny sensor objects
Adafruit_BME280 bme[3];
Adafruit_SHT31 sht31[3];
ClosedCube_HDC1080 hdc1080[2];
Adafruit_HTU21DF htu[3];

//particulate matter objects
SoftwareSerial soft[3]={SoftwareSerial(10,11),SoftwareSerial(12,13),SoftwareSerial(62,63)};
SdsDustSensor sds[3]={SdsDustSensor(soft[0]),SdsDustSensor(soft[1]),SdsDustSensor(soft[2])}; //passing Softwareserial& as parameter
SerialPM pmsa_array[3]={SerialPM(PMSA003, Serial1),SerialPM(PMSA003, Serial2),SerialPM(PMSA003, Serial3)};// passing HardwareSerial& as parameter

//small sensor status established in setup function
unsigned statusBME[3];
unsigned statusSHT[3];
unsigned statusHTU[3];

//data strings that will be written to the sdcard
String timeStamp="";
String tempString="";
String humString="";
String airPartString="";

//for the soil moisture and temp sensor
const int SMTmeasurements = 50;// multiple measurements to reduce noise error
float SMTtemp=0.0;
float SMTmois=0.0;

// function to select channels
void Tcselect(uint8_t bus){
  
  Wire.beginTransmission(0x70);
  Wire.write(1<<bus);
  Wire.endTransmission();
}

////// Reading fucntion for the small sensors /////
void Readdata(int i){
  // the channel corresponds to the i so we shall use the same variable
  Tcselect(i+1);
  //check if sensor is connected if not it will cause the system to reset-- i don't know why yet
  // if the sensor is not connected, then its reading will be zero, that is under else block to be added.
  if (statusSHT[i]){
    tempString+=(String(sht31[i].readTemperature())+",");
    humString+=(String(sht31[i].readHumidity())+",");
  }else{
    tempString+=("NULL,");
    humString+=("NULL,");
    }
  if (statusBME[i]){
    tempString+=(String(bme[i].readTemperature())+",");
    humString+=(String(bme[i].readHumidity())+",");
  }else{
    tempString+=("NULL,");
    humString+=("NULL,");
    }
  if (statusHTU[i]){
    tempString+=(String(htu[i].readTemperature())+",");
    humString+=(String(htu[i].readHumidity())+",");
    
  }else{
    tempString+=("NULL,");
    humString+=("NULL,");
    }
  if (i<2){// we only have 2 of these
    //We have to make sure they are connected otherwise since we have no status check for these yet.
    // Their ".begin()" doesn't return anything
    tempString+=(String(hdc1080[i].readTemperature())+",");
    humString+=(String(hdc1080[i].readHumidity())+",");
  }
  
}

void setup() {
 //initialize all the sensors
  Wire.begin();
  rtc.begin();
  for(int i=0;i<3;i++){
    //small sensors
    Tcselect(i+1);// select channel 
    statusHTU[i] = htu[i].begin();
    statusSHT[i] = sht31[i].begin(); 
    statusBME[i] =bme[i].begin(0x76); 
    if(i<2){
      hdc1080[i].begin(0x40);// doesn't return boolean so just intialize 
    }
    //big sensors
    pmsa_array[i].init();
    sds[i].begin(); // this line will begin soft-serial with given baud rate (9600 by default)
    sds[i].setQueryReportingMode(); // set reporting mode  
    }
    //sdcard
    pinMode(CSpin, OUTPUT);
    if (!SD.begin(CSpin)) {
      //put an indicator led
    }
}

void loop() {
//overrite the last strings 
  tempString="";
  humString="";
  airPartString="";
  // wake up the big sensors
  for(int i=0;i<3;i++){
    soft[i].listen();
    sds[i].wakeup();
    //need to include the set pin for the pmsa003 sleep and wakeup
    }
    delay(40000);

  //--reading the SMT---
  for (int i = 0; i < SMTmeasurements; i++)
  {
    SMTtemp += analogRead(A0);
    SMTmois += analogRead(A1);
  }
  SMTtemp= (SMTtemp/SMTmeasurements)*5/1024;
  SMTmois= (SMTmois/SMTmeasurements)*5/1024;
  SMTtemp = (SMTtemp-0.5)*100;
  SMTmois = SMTmois*50/3;
  //tempString+=(String(SMTtemp)+",");
  
  ///major loop here tor read all the other sensors that are in 3s
  timeStamp=(String(rtc.getDateStr())+"-"+String(rtc.getTimeStr()));
  for(int i=0;i<3;i++){
    Readdata(i);// first read from the small sensors
    soft[i].listen();
    PmResult pm = sds[i].queryPm();
  if (pm.isOk()) {
    airPartString+=(String(pm.pm25)+","+String(pm.pm10)+",");
  } else {
    airPartString+=("NULL,NULL,");
  }
  
  //sleep after reading an sds.
     WorkingStateResult state = sds[i].sleep();
  if (!state.isWorking()) {
    //indicator led for sleeping
  }
   
  delay(1000);
  
  pmsa_array[i].read();
    if (pmsa_array[i]) {
      // print results
      // instead of printing, should be sending the data to
      // a responsible sd card location in the format required.

      // for particles less than 1.0ug/m3
      airPartString+=(String(pmsa_array[i].pm01)+","+String(pmsa_array[i].pm25)+","+String(pmsa_array[i].pm10)+",");
    /*
      Don't know if these other readings are needed from the pmsa003
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
      airPartString+=("NULL,NULL,NULL,");
      //put here led indicator 
      
    }
  }
  
  tempString+=timeStamp;
  humString+=timeStamp;
  airPartString+=timeStamp;
  saveData(tempFile,tempString,"temp.csv");
  delay(1000);
  saveData(humFile,humString,"hum.csv");
  delay(1000);
  saveData(airPartFile,airPartString,"airpart.csv");
  
delay (30000);
}
void saveData(File sensorData, String Data ,String filename){
  //assumes already the file with that name already exists on the card
if(SD.exists(filename)){ // check the card is still there
// now append new data file
sensorData = SD.open(filename, FILE_WRITE);
if (sensorData){
sensorData.println(Data);
sensorData.close(); // close the file
}
}
else{
//Serial.println("Error writing to file !"); place indicator led
}
}
