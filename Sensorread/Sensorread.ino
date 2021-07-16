#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include<SPI.h>
#include "Adafruit_SHT31.h"
#include "ClosedCube_HDC1080.h"
#include "Adafruit_HTU21DF.h"


Adafruit_BME280 bme[3];

Adafruit_SHT31 sht31[3];

ClosedCube_HDC1080 hdc1080[3];

Adafruit_HTU21DF htu[3];

const int SMTmeasurements = 50;// multiple measurements to reduce noise error
float SMTtemp=0.0;
float SMTmois=0.0;
float BMEtmp[3]={};
float BMEhum[3]={};
float SHTtmp[3]={};
float SHThum[3]={};
float HDCtmp[3]={};
float HDChum[3]={};
float HTUtmp[3]={};
float HTUhum[3]={};

// function to select channels
void Tcselect(uint8_t bus){
  
  Wire.beginTransmission(0x70);
  Wire.write(1<<bus);
  Wire.endTransmission();
}
////// Initialization functions
// bme intialization
void BmeIntialize(Adafruit_BME280 &bme ,char *name){
  unsigned status;
  status = bme.begin(0x76); 
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring for:");
    Serial.print(name);
    while (1) delay(10);
  }
}

//sht initalization
void Shtinitalize(Adafruit_SHT31 &sht31, char *name){
  unsigned status;
  status = sht31.begin(); 
  if (!status) {
    Serial.println("Could not find a valid Sht sensor, check wiring for:");
    Serial.print(name);
    while (1) delay(10);
  }
}

//Htu initalization
void Htuinitialize(Adafruit_HTU21DF &htu21df, char* name){
  unsigned status;
  status = htu21df.begin(); 
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring for:");
    Serial.print(name);
    while (1) delay(10);
  }
}


////// Reading fucntion /////
void Readdata(int index){
  // the channel corresponds to the index so we shall use the same variable
  Tcselect(index+1);
  SHTtmp[index]=sht31[index].readTemperature();
  SHThum[index]=sht31[index].readHumidity();
  BMEtmp[index]=bme[index].readTemperature();
  BMEhum[index]=bme[index].readHumidity();
  HTUtmp[index]=htu[index].readTemperature();
  HTUhum[index]=htu[index].readHumidity();
  HDCtmp[index]=hdc1080[index].readTemperature();
  HDChum[index]=hdc1080[index].readHumidity();

  //---SERIAL OUTPUT TO BE COMMENTED OUT BEFORE DEPLOYMENT-----
  if (! isnan(SHTtmp[index])) {  // check if 'is not a number'
    Serial.print("SHT_Temp *C = "); Serial.print(SHTtmp[index]); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read Sht temperature");
  }
  
  if (! isnan(SHThum[index])) {  // check if 'is not a number'
    Serial.print("SHT_Hum. % = "); Serial.println(SHThum[index]);
  } else { 
    Serial.println("Failed to read sht humidity");
  }
  

  if (! isnan(HTUtmp[index])) {  // check if 'is not a number'
    Serial.print("HTU_Temp *C = "); Serial.print(HTUtmp[index]); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read Sht temperature");
  }
  
  if (! isnan(HTUhum[index])) {  // check if 'is not a number'
    Serial.print("HTU_Hum. % = "); Serial.println(HTUhum[index]);
  } else { 
    Serial.println("Failed to read htu humidity");
  }
// we only have 2 hdc
  if(index<2){
    if (! isnan(HDCtmp[index])) {  // check if 'is not a number'
    Serial.print("HDC_Temp *C = "); Serial.print(HDCtmp[index]); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read hdc temperature");
  }
  
  if (! isnan(HDChum[index])) {  // check if 'is not a number'
    Serial.print("HDC_Hum. % = "); Serial.println(HDChum[index]);
  } else { 
    Serial.println("Failed to read HDC humidity");
  }

  }

  if (! isnan(BMEtmp[index])) {  // check if 'is not a number'
    Serial.print("BME_Temp *C = "); Serial.print(BMEtmp[index]); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read BME temperature");
  }
  
  if (! isnan(SHThum[index])) {  // check if 'is not a number'
    Serial.print("BME_Hum. % = "); Serial.println(BMEhum[index]);
  } else { 
    Serial.println("Failed to read BME humidity");
  }
  Serial.println("-----------------------------------------");

  delay(1000);
  
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Measument Readings");
  
  //------------check status as initializing------------
  Tcselect(1);// select channel 1
  BmeIntialize(bme[0]," bme1");
  Shtinitalize(sht31[0],"sht31_1");
  hdc1080[0].begin(0x40);// doesn't return boolean so just intialize
  Htuinitialize(htu[0]," Htu21df_1");
  // other sensor on channel 1 initialized here
  /*

  Tcselect(2);// select channel 2
  BmeIntialize(bme[1]," bme2");
  Shtinitalize(sht31[1],"sht31_2");
  hdc1080[1].begin(0x40);
  Htuinitialize(htu[1]," Htu21df_2");
  // other sensor on channel 2 initialized here

  Tcselect(3);// select channel 3
  BmeIntialize(bme[2]," bme3");
  Shtinitalize(sht31[2],"sht31_3");
  hdc1080[2].begin(0x40);
  Htuinitialize(htu[2]," Htu21df_3");
  // other sensor on channel 3 initialized here
  */

  
}

void loop() {
  //--reading the SMT---
  for (int i = 0; i < SMTmeasurements; i++)
  {
    SMTtemp += analogRead(A0);
    SMTmois +=analogRead(A1);
  }
  SMTtemp= (SMTtemp/SMTmeasurements)*5/1024;
  SMTmois= (SMTmois/SMTmeasurements)*5/1024;

  SMTtemp = (SMTtemp-0.5)*100;
  SMTmois = SMTmois*50/3;
  
  Serial.print("STM_temp *C = "); Serial.print(SMTtemp);Serial.print("\t\t");
  Serial.print("STM_mois. % = "); Serial.println(SMTmois);
  Readdata(0);

  
  //Readdata(1);
  //Readdata(2);
  // put your main code here, to run repeatedly:

}
