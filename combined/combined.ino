#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include "SD.h"
#include "Adafruit_SHT31.h"
#include "Adafruit_HTU21DF.h"
#include "SdsDustSensor.h"
#include <PMserial.h>
#include "SoftwareSerial.h"
#include <DS3231.h>
#include "hdc.h"
#include <avr/wdt.h>

DS3231  rtc(SDA, SCL);

//sdcard
const int CSpin = 53;
File tempFile;
File humFile;
File airPartFile;
File logsfile;

//tiny sensor objects
Adafruit_BME280 bme[3];
Adafruit_SHT31 sht31[3];
HDC1080 hdc1080[2];
Adafruit_HTU21DF htu[3];

//particulate matter objects
SoftwareSerial soft[3]={SoftwareSerial(10,11),SoftwareSerial(12,13),SoftwareSerial(62,63)};
SdsDustSensor sds[3]={SdsDustSensor(soft[0]),SdsDustSensor(soft[1]),SdsDustSensor(soft[2])}; //passing Softwareserial& as parameter
SerialPM pmsa_array[2]={SerialPM(PMSA003, Serial1),SerialPM(PMSA003, Serial2)};// passing HardwareSerial& as parameter

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

int error{0};
ISR (WDT_vect)
{
  if (error ==1){
    //Serial.println("error setting up 12c sensors");
    saveData(logsfile,"error setting up i2c sensors","logs.txt");
    //error ==0;
    }
  if (error ==2){
    //Serial.println("error reading i2c sensors");
    saveData(logsfile,"error reading i2c sensors","logs.txt");
    //error ==0;
    }
  if (error ==3){
    //Serial.println("error reading time");
    saveData(logsfile,"error reading time","logs.txt");
    //error ==0;
    }
  if (error ==4){
    //Serial.println("error reading i2c sensors");
    saveData(logsfile,"error setting up rtc","logs.txt");
    //error ==0;
    delay(1000);
    // Add your own funky code here  (but do keep it short and sweet).
}
}

// function to select channels
void Tcselect(uint8_t bus){
  
  Wire.beginTransmission(0x70);
  Wire.write(1<<bus);
  Wire.endTransmission();
}

int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

////// Reading fucntion for the small sensors /////
void Readdata(int i){
  error =2;
  wdt_enable( WDTO_8S);
  WDTCSR |= (1 << WDIE);  // Watchdog Interrupt Enable
  WDTCSR |= (1 << WDE); // Watchdog  System Reset Enable
  // the channel corresponds to the i so we shall use the same variable
  Tcselect(i+2);
  delay(100);
  //check if sensor is connected if not it will cause the system to reset-- i don't know why yet
  // if the sensor is not connected, then its reading will be zero, that is under else block to be added.
  if (statusSHT[i]){
    tempString+=(String(sht31[i].readTemperature())+",");
    delay(100);
    humString+=(String(sht31[i].readHumidity())+",");
    delay(100);
  } else {
    tempString+=("NULL,");
    humString+=("NULL,");
  }
  if (statusBME[i]){
    tempString+=(String(bme[i].readTemperature())+",");
    delay(100);
    humString+=(String(bme[i].readHumidity())+",");
    delay(100);
  } else {
    tempString+=("NULL,");
    humString+=("NULL,");
  }
  if (statusHTU[i]){
    tempString+=(String(htu[i].readTemperature())+",");
    delay(100);
    humString+=(String(htu[i].readHumidity())+",");
    delay(100);
    
  }else{
    tempString+=("NULL,");
    humString+=("NULL,");
  }
  if (i<2){// we only have 2 of these
    Tcselect(i);
    //We have to make sure they are connected otherwise since we have no status check for these yet.
    // Their ".begin()" doesn't return anything
    tempString+=(String(hdc1080[i].getTemperature())+",");
    delay(100);
    humString+=(String(hdc1080[i].getHumidity())+",");
    delay(100);
  }
  wdt_disable();
}

void setup() {
  Serial.begin(9600);
   int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    //Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      //Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      //Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      //Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }
  delay(9000);
 //initialize all the sensors
  //Wire.begin();
  delay(100);
  error =4;
  wdt_enable( WDTO_8S);
  WDTCSR |= (1 << WDIE);  // Watchdog Interrupt Enable
  WDTCSR |= (1 << WDE); // Watchdog  System Reset Enable
  rtc.begin();
  wdt_disable();
  delay(100);
  Serial.println("after begin");
  error =1;
  wdt_enable( WDTO_8S);
  WDTCSR |= (1 << WDIE);  // Watchdog Interrupt Enable
  WDTCSR |= (1 << WDE); // Watchdog  System Reset Enable
  for(int i=0;i<3;i++){
    //small sensors
    Tcselect(i+2);// select channel 
    delay(100);
    statusHTU[i] = htu[i].begin();
    delay(100);
    statusSHT[i] = 1;
    sht31[i].begin(); 
    delay(100);
    statusBME[i] =bme[i].begin(0x76); 
    delay(100);
    Serial.println("before begin");
    if(i<2){
      Tcselect(i);
      delay(100);
      hdc1080[i].begin();// doesn't return boolean so just intialize 
      delay(100);
    }
  }
  wdt_disable();
  for(int i=0;i<3;i++){
    //big sensors
    if (i < 2) {
      pmsa_array[i].init();
    }
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
    Serial.println("waking up");
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
  error =3;
  wdt_enable( WDTO_8S);
  WDTCSR |= (1 << WDIE);  // Watchdog Interrupt Enable
  WDTCSR |= (1 << WDE); // Watchdog  System Reset Enable
  timeStamp=(String(rtc.getDateStr())+"-"+String(rtc.getTimeStr()));
  wdt_disable();
  //erial.println(timeStamp);
  //timeStamp="NULL";
  for(int i=0;i<3;i++){
    Readdata(i);// first read from the small sensors
    soft[i].listen();
    PmResult pm = sds[i].queryPm();
    if (pm.isOk()) {
      airPartString+=(String(pm.pm25)+","+String(pm.pm10)+",");
    } else {
      airPartString+=("NULL,NULL,");
    }
    Serial.println("food");
    //sleep after reading an sds.
     WorkingStateResult state = sds[i].sleep();
    if (!state.isWorking()) {
      //indicator led for sleeping
    }
   
    delay(1000);

    if (i < 2) {
      pmsa_array[i].read();
      delay(1000);
      if (pmsa_array[i]) {
        
        
        airPartString+=(String(pmsa_array[i].pm01)+","+String(pmsa_array[i].pm25)+","+String(pmsa_array[i].pm10)+",");
        
      } else {
        airPartString+=("NULL,NULL,NULL,");
        //put here led indicator 
        
      }
    }
  }
  
  tempString+=(String(SMTtemp)+","+timeStamp);
  humString+=(String(SMTmois)+","+timeStamp);
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
  }else{
    //Serial.println("Error writing to file !"); place indicator led
  }
}
