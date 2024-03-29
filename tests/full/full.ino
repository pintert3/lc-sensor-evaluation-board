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
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>

//////gsm
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#define SerialMon Serial
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650 // for response
#endif
#define TINY_GSM_DEBUG SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 9600
#define pinReset 6
SoftwareSerial SerialAT(68, 69);  // RX, TX

DS3231  rtc(SDA, SCL);

//sdcard
const int CSpin = 53;
const int SD_CARD_LED = 47;

// timing
const unsigned long DATA_SEND_TIME = 120000;
const unsigned long PERIOD = 300000;
volatile unsigned long startTime = 0;
//volatile uint8_t fresh_reset = 1;

// data formatting and storage
const unsigned int FILE_LINE_LENGTH = 520;
const String dataFile = String("data.txt");

// data age
const uint8_t NEW_DATA = 1;
const uint8_t OLD_DATA = 0;
uint8_t OLD_DATA_AVAILABLE = 1;

// File dataFile;
// File logsfile;

//tiny sensor objects
Adafruit_BME280 bme[3];
Adafruit_SHT31 sht31[3];
HDC1080 hdc1080[3];
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

//for the soil moisture and temp sensor
const int SMTmeasurements = 50;// multiple measurements to reduce noise error
float SMTtemp=0.0;
float SMTmois=0.0;


// network  connection
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";


//communication variables
const char server[]   = "35.226.209.188";
char resource[]="/api_v1/general/";//endpoint will be hard written here
String contentType ="application/json";
const int  port       = 80;//port http

TinyGsm        modem(SerialAT);

TinyGsmClient client(modem);
HttpClient          http(client, server, port);
volatile int counter; //delay counter     
volatile int countmax = 3; 

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
void readData(int i,StaticJsonDocument<1024>& doc){
  JsonArray data;
  Serial.println("-- Enable WDT in readData");
  wdt_enable( WDTO_8S);
  // the channel corresponds to the i so we shall use the same variable
  Tcselect(i+1);
  delay(100);
  //check if sensor is connected if not it will cause the system to reset-- i don't know why yet
  // if the sensor is not connected, then its reading will be zero, that is under else block to be added.
  data = doc.createNestedArray(String("sht_")+String(i+1));
  if (statusSHT[i]){
    data.add(sht31[i].readTemperature());
    delay(100);
    data.add(sht31[i].readHumidity());
    delay(100);
  } else {
    data.add("NULL");
    data.add("NULL");
  }
  Serial.println("* Read from SHT *");
  
  data = doc.createNestedArray(String("bme_")+String(i+1));
  if (statusBME[i]){
    data.add(bme[i].readTemperature());
    delay(100);
    data.add(bme[i].readHumidity());
    delay(100);
  } else {
    data.add("NULL");
    data.add("NULL");
  }
  Serial.println("* Read from BME *");

  data = doc.createNestedArray(String("htu_")+String(i+1));
  htu[i].readHumidity(); // just to make it work
  data.add(htu[i].readTemperature());
  delay(100);
  data.add(htu[i].readHumidity());
  delay(100);
  Serial.println("* Read from HTU *");

  // if (i<2){// we only have 2 of these
  data = doc.createNestedArray(String("hdc_")+String(i+1));
  Tcselect(i+4);
  //We have to make sure they are connected otherwise since we have no status check for these yet.
  // Their ".begin()" doesn't return anything
  data.add(hdc1080[i].getTemperature());
  delay(100);
  data.add(hdc1080[i].getHumidity());
  delay(100);
  Serial.println("* Read from HDC *");
  // }
  wdt_disable();
  Serial.println("-- Disabled WDT in readData");

  soft[i].listen();
  data = doc.createNestedArray(String("sds_")+String(i+1));
  PmResult pm = sds[i].queryPm();
  if (pm.isOk()) {
    data.add(pm.pm25);
    data.add(pm.pm10);
  } else {
    
    data.add("NULL");
    data.add("NULL");
  }
  //sleep after reading an sds.
   WorkingStateResult state = sds[i].sleep();
  if (!state.isWorking()) {
    //indicator led for sleeping
  }
 
  delay(1000);

  if (i < 2) {
    data = doc.createNestedArray(String("pmsa_")+String(i+1));
    pmsa_array[i].read();
    delay(1000);
    if (pmsa_array[i]) {
      
      
      
      data.add(pmsa_array[i].pm01);
      data.add(pmsa_array[i].pm25);
      data.add(pmsa_array[i].pm10);
      
    } else {
      data.add("NULL");
      data.add("NULL");
      data.add("NULL");
      //put here led indicator 
      
    }
    pmsa_array[i].sleep();
  }
}

void setup() {
  startTime = millis();

  //SD_CARD_LED
  pinMode(SD_CARD_LED, OUTPUT);
  SerialMon.begin(115200);
  SerialMon.println("========= RESET =========");
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
  Serial.println("-- Enable WDT");
  wdt_enable( WDTO_8S);
  rtc.begin();      //  <-------------- DEBUG
  wdt_disable();
  Serial.println("-- Disabled WDT");
  delay(100);
  //Serial.println("after begin");
  Serial.println("-- Enable WDT");
  wdt_enable( WDTO_8S);
  for(int i=0;i<3;i++){
    //small sensors
    Tcselect(i+1);// select channel 
    Serial.print("---- Initializing channel ");
    Serial.println(i+1);
    Serial.println("Initializing HTU");
    delay(100);
    statusHTU[i] = htu[i].begin();
    if (statusHTU[i]) {
      Serial.println("<- Initialized HTU");
    } else {
      Serial.println("<-xx HTU NOT FOUND");
    }
    Serial.println("Initializing SHT");
    delay(100);
    statusSHT[i] = 1;
    sht31[i].begin(); 
    if (statusSHT[i]) {
      Serial.println("<- Initialized SHT");
    } else {
      Serial.println("<-xx SHT NOT FOUND");
    }
    Serial.println("Initializing BME");
    delay(100);
    statusBME[i] =bme[i].begin(0x76); 
    if (statusBME[i]) {
      Serial.println("<- Initialized BME");
    } else {
      Serial.println("<-xx BME NOT FOUND");
    }
    delay(100);
    //Serial.println("before begin");

    Tcselect(i+4);
    delay(100);
    hdc1080[i].begin();// doesn't return boolean so just intialize 
    Serial.println("<- Initialized HDC1080");
    delay(100);
  }
  wdt_disable();
  Serial.println("-- Disabled WDT");
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
    startTime = millis();
  
  SerialMon.println(millis());
  
  char payload[1024] = {0};
  StaticJsonDocument<1024> doc;
//overrite the last strings 
  // wake up the big sensors
  for(int i=0;i<3;i++){
    soft[i].listen();
    sds[i].wakeup();
    delay(800);
    if (i<2) {
      pmsa_array[i].wake();
    }
  }
  //Serial.println("waking up");
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
 
  ///major loop here tor read all the other sensors that are in 3s
  wdt_enable( WDTO_8S);
  //        <-------------- DEBUG
  timeStamp=(String(rtc.getDateStr())+"-"+String(rtc.getTimeStr()));
  wdt_disable();
  
  for(int i=0;i<3;i++){
    readData(i,doc);// first read from the small sensors
  }
  SerialMon.println(millis());
  JsonArray data = doc.createNestedArray("soil");
  data.add(SMTtemp);
  data.add(SMTmois);
  //         <-------------- DEBUG
  doc["timestamp"] = timeStamp;

  serializeJson(doc, payload);

  Serial.println("==== > SENT DATA < ====");
  Serial.println(payload);
  
  char dataToSave[FILE_LINE_LENGTH+1];
  strcpy(dataToSave, payload); // should copy the first 512 bytes + 0

  formatData(dataToSave);
  digitalWrite(SD_CARD_LED,HIGH);
  saveData(dataToSave, dataFile);
  SerialMon.println(millis());
  setupgsm();
  connectnet();

  // how to change OLD_DATA_AVAILABLE to true?
  if (sendData(payload, NEW_DATA)) {
    if (OLD_DATA_AVAILABLE) {
      if (timeLeft() > DATA_SEND_TIME) {
        memset(payload, 0, 1024);
        if (readOldData(payload, dataFile)) {
          sendData(payload, OLD_DATA);
        } else {
          // in case there's no data or an error in reading, 
          // do nothing.
        }
        // timeLeft = PERIOD - (millis() - startTime);
      }
    }
  }

  digitalWrite(SD_CARD_LED,LOW);
  http.stop();
  SerialMon.println(F("Server disconnected"));
  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
  
  while (timeLeft() > 0);
}

void saveData(char Data[FILE_LINE_LENGTH+1] ,String filename){
  //assumes already the file with that name already exists on the card
  if(SD.exists(filename)){ // check the card and file is there
    // now append new data file
    File sensorData = SD.open(filename, FILE_WRITE);
    if (sensorData){
      sensorData.write('!');
      sensorData.write(Data);  // DO NOT use println, unless you want <CR>
      sensorData.write('\n');  // Check if Data has an end \0
      sensorData.close(); // close the file
    }
  }else{
    //Serial.println("Error writing to file !"); place indicator led
    // maybe return something?
  }
}
//gsm functions

void setupgsm(){
  pinMode(pinReset ,OUTPUT);//reset pin
  // Set console baud rate
  //SerialMon.begin(115200);
  delay(10);
  digitalWrite(pinReset, HIGH);
  delay(500);
  digitalWrite(pinReset, LOW);
  delay(500);
  digitalWrite(pinReset, HIGH);
  delay(3000);

  SerialMon.println("Wait...");
  //modem.setBaud(9600);
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  delay(6000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
   //modem.init();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
}


 void connectnet(){
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    watchdogEnable(); 
    while(true);  
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) { 
    SerialMon.println("Network connected");
  }
    SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    watchdogEnable(); 
      while(true);
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected"); 
  }
  
  delay(5000);
}


void watchdogEnable()
{
  countmax = (timeLeft()/8000)-3;
  counter=0;
  cli();                             
  MCUSR = 0;                                                                                             
  WDTCSR |= 0b00011000;                                                  
  WDTCSR =  0b01000000 | 0b100001;                                        
  sei();                              
}
ISR(WDT_vect) 
{
  counter+=1;
  if (counter < countmax)
  {
    wdt_reset(); 
  }
  else             
  {
    MCUSR = 0;                          // reset flags                            
    WDTCSR |= 0b00011000;               
    WDTCSR =  0b00001000 | 0b000000;    
    
  }
}

int sendData(char* postData, uint8_t age) {
  watchdogEnable(); // In case connection to server fails
  SerialMon.print(F("Performing HTTP POST request... "));
  //http.connectionKeepAlive();  // Currently, this is needed for HTTPS
  //int err = http.get(resource);
  int err = http.post(resource,contentType,postData);//even if they are char arrays
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    while(true); // To be caught by WDT
  }
  wdt_disable();
  Serial.println("**** starting loop ****");
  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if ((status != 200) && (status != -3)) {
    return 0;
  }

  watchdogEnable(); 
  if (status == -3) {
    markData(age, dataFile);
    digitalWrite(SD_CARD_LED,LOW);
    http.stop();
    SerialMon.println(F("Server disconnected"));
    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
    while(1) {}
  }
  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    //SerialMon.println("    " + headerName + " : " + headerValue);
  }
  int length = http.contentLength();
  if (length >= 0) {
    //SerialMon.print(F("Content length is: "));
    //SerialMon.println(length);
  }
  if (http.isResponseChunked()) {
    //SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  //SerialMon.println(F("Response:"));
  //SerialMon.println(body);

  //SerialMon.print(F("Body length is: "));
  //SerialMon.println(body.length());
  wdt_disable();
  
  markData(age, dataFile);
  return 1;
}


void formatData(char *input) {
  // formats input char string to fixed length output with \t at end padded
  // with zero characters ('0')

  int original_length = strlen(input);
  if (original_length < FILE_LINE_LENGTH) {
    input[original_length] = '\t';
    for (int i = original_length+1; i < FILE_LINE_LENGTH; i++) {
      input[i] = '0';
    }
    input[FILE_LINE_LENGTH] = '\0'; // end with 0
  }
}

int readOldData(char* output, String filename){
  // Should return 1 if old data is found
  // else, it should return 0
  // NOTE: Could output error codes instead.

  char nextChar = -1;

  if(SD.exists(filename)){ 
    char buffer[FILE_LINE_LENGTH];

    // open data file if opening went well
    File sensorData = SD.open(filename, FILE_READ);
    if (sensorData) { // if file was able to open
      nextChar = sensorData.peek();
      while (nextChar >= 0) {
        if (nextChar == '!') {
          sensorData.seek(sensorData.position()+1);
          sensorData.read(buffer, FILE_LINE_LENGTH);
          prepare_payload(buffer, output); // can simply read to output
          break;
        } else if (nextChar == '?') {
          sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2);
          nextChar = sensorData.peek();
        } else { // formatting error
          return 0;
        }
      }

      sensorData.close(); // close the file

      if (nextChar < 0) {
        OLD_DATA_AVAILABLE = 0;
        return 0;
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }else{
    //Serial.println("Error writing to file !"); place indicator led
    return 0;
  }

}

void prepare_payload(char* unfiltered_data, char* output) {
  // loop forwards the array of length FILE_LINE_LENGTH
  // until find \t character.
  // send that to payload
  int i;
  for (i = 0; ((i < FILE_LINE_LENGTH) && (unfiltered_data[i] != '\t')); i++) {
    output[i] = unfiltered_data[i];
  }
  output[i] = 0;
}


int markData(uint8_t age, String filename) {
  int output_code = 1;
  char nextChar;
  if(SD.exists(filename)){
    // open data file
    File sensorData = SD.open(filename, O_RDWR);
    if (sensorData){
      if (age == NEW_DATA) {
        // seek back to start of last line and add '?'
        sensorData.seek(sensorData.size());
        sensorData.seek(sensorData.position()+((int)FILE_LINE_LENGTH*-1) - 2); 
        sensorData.write('?');
        output_code = 0;
      } else { // Marking OLD_DATA
        // sensorData.seek(0);
        nextChar = sensorData.peek();
        while (nextChar != -1) { // not expecting zero(0)
          if (nextChar == '!') {
            sensorData.write('?');
            output_code = 0;
            break;
          } else if (nextChar == '?') {
            sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2); // Skip the ? and the \n
            nextChar = sensorData.peek();
          } else {
            output_code = 2; // means error with data format(arrangement)
          }
        }
      }
      sensorData.close();
    }else{
      output_code = 3;
    }
  } else {
    output_code = 4;
  }
  return output_code;
}

unsigned long timeLeft() {
  return (timeElapsed() < PERIOD) ? PERIOD - timeElapsed() : 0;
}

unsigned long timeElapsed() {
  return (millis() - startTime);
}
