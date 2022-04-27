#include <Wire.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include "SD.h"
#include "Adafruit_SHT31.h"
#include "Adafruit_HTU21DF.h"
#include "SdsDustSensor.h"
// #include <PMserial.h>
#include "SoftwareSerial.h"
#include <DS3231.h>
#include "hdc.h"
#include <avr/wdt.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>

// DEBUG MODE if needed
// #ifndef DEBUG_MODE
// #define DEBUG_MODE
// #endif

//// TCA CHANNELS
#define TCA_1 1
#define TCA_2 4

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
#define pinReset 4
SoftwareSerial SerialAT(2, 3);  // RX, TX

DS3231  rtc;

// LEDs
const int SD_CARD_LED = 10;
const int SENSOR_LED = 9;

// timing
const unsigned long DATA_SEND_TIME = 120000;
const unsigned long PERIOD = 300000;
volatile unsigned long startTime = 0;
//volatile uint8_t fresh_reset = 1;

// data formatting and storage
const unsigned int FILE_LINE_LENGTH = 520;
const String dataFile = String("data.txt");

//--- SD CARD VARIABLES

// spi chip select pin
const int CSpin = 8;

// data age
const uint8_t NEW_DATA = 1;
const uint8_t OLD_DATA = 0;
uint8_t OLD_DATA_AVAILABLE = 1;

//----

// File dataFile;
// File logsfile;

//tiny sensor objects
Adafruit_BME280 bme;
Adafruit_SHT31 sht31;
HDC1080 hdc1080;
Adafruit_HTU21DF htu;

//particulate matter objects
SoftwareSerial novaSoft = SoftwareSerial(5,6);
SdsDustSensor sds = SdsDustSensor(novaSoft); //passing Softwareserial& as parameter
// SerialPM pmsa_array[2]={SerialPM(PMSA003, Serial1),SerialPM(PMSA003, Serial2)};// passing HardwareSerial& as parameter

//small sensor status established in setup function
unsigned statusBME;
unsigned statusSHT;
unsigned statusHTU;


//for the soil moisture and temp sensor
const int SMTmeasurements = 50;// multiple measurements to reduce noise error
float SMTtemp=0.0;
float SMTmois=0.0;


// // network  connection
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

int softrst=0;
//communication variables
// const char server[]   = "35.226.209.188";
const char server[]   = "137.63.184.136";
// char resource[]="/api_v1/general/";//endpoint will be hard written here
// char contentType[] ="application/json";
const int  port       = 8000;//port http

TinyGsm        modem(SerialAT);

TinyGsmClient client(modem);
HttpClient          http(client, server, port);

volatile int counter; //delay counter     
volatile int countmax = 3; 

// ADAFruit sensor templates
// TODO: Use templates to refactor adafruit sensor code

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

////// Reading function for the small sensors /////

void readData(StaticJsonDocument<1024>& doc){
  JsonArray data;
  // wdt_enable( WDT_PERIOD_8KCLK_gc);
  // the channel corresponds to the i so we shall use the same variable
  Tcselect(TCA_1);
  delay(100);
  //check if sensor is connected if not it will cause the system to reset-- i don't know why yet
  // if the sensor is not connected, then its reading will be zero, that is under else block to be added.
  data = doc.createNestedArray("sht");
  if (statusSHT){
    data.add(sht31.readTemperature());
    delay(100);
    data.add(sht31.readHumidity());
    delay(100);
  } else {
    data.add("NULL");
    data.add("NULL");
  }
  
  data = doc.createNestedArray("bme");
  if (statusBME){
    data.add(bme.readTemperature());
    delay(100);
    data.add(bme.readHumidity());
    delay(100);
  } else {
    data.add("NULL");
    data.add("NULL");
  }

  data = doc.createNestedArray("htu");
  htu.readHumidity(); // just to make it work
  data.add(htu.readTemperature());
  delay(100);
  data.add(htu.readHumidity());
  delay(100);

  data = doc.createNestedArray("hdc");
  Tcselect(TCA_2);
  //We have to make sure they are connected otherwise since we have no status check for these yet.
  // Their ".begin()" doesn't return anything
  data.add(hdc1080.getTemperature());
  delay(100);
  data.add(hdc1080.getHumidity());
  delay(100);
  // }
  // wdt_disable();

  novaSoft.listen();
  data = doc.createNestedArray("sds");
  PmResult pm = sds.queryPm();
  if (pm.isOk()) {
    data.add(pm.pm25);
    data.add(pm.pm10);
  } else {
    
    data.add("NULL");
    data.add("NULL");
  }
  //sleep after reading an sds.
   WorkingStateResult state = sds.sleep();
 
  delay(1000);

}

void setup() {
  rtcwdt();
  startTime = millis();

  //SD_CARD_LED
  pinMode(SD_CARD_LED, OUTPUT);
  #ifdef DEBUG_MODE
  SerialMon.begin(115200);
  SerialMon.println("========= RESET =========");
  #endif
  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  // SUBJECT TO CODE REFACTORING
  // if (rtn != 0) {
  //   //Serial.println(F("I2C bus error. Could not clear"));
  //   if (rtn == 1) {
  //     //Serial.println(F("SCL clock line held low"));
  //   } else if (rtn == 2) {
  //     //Serial.println(F("SCL clock line held low by slave clock stretch"));
  //   } else if (rtn == 3) {
  //     //Serial.println(F("SDA data line held low"));
  //   }
  // } else { // bus clear
  //   // re-enable Wire
  //   // now can start Wire Arduino master
  //   Wire.begin();
  // }

  if (rtn == 0) {
    Wire.begin();
  }

  delay(9000);
  //initialize all the sensors
  // delay(100);
  // wdt_enable( WDT_PERIOD_8KCLK_gc);

  // rtc.begin();

  // wdt_disable();
  // delay(100);
  //Serial.println("after begin");
  // wdt_enable( WDT_PERIOD_8KCLK_gc);

  //small sensors
  Tcselect(TCA_1);// select channel 
  delay(100);
  statusHTU = htu.begin();
  delay(100);
  statusSHT = 1;
  sht31.begin(); 
  delay(100);
  statusBME = bme.begin(0x76); 
  delay(100);
  //Serial.println("before begin");

  Tcselect(TCA_2);
  delay(100);
  hdc1080.begin();// doesn't return boolean so just intialize 
  delay(100);

  // wdt_disable();

  //nova
  sds.begin(); // this line will begin soft-serial with given baud rate (9600 by default)
  sds.setQueryReportingMode(); // set reporting mode  

    //sdcard
    pinMode(CSpin, OUTPUT);
    if (!SD.begin(CSpin)) {
      //put an indicator led
    }

}

void loop() {
  startTime = millis();
  
  #ifdef DEBUG_MODE
  SerialMon.println(millis());
  #endif
  
  char payload[1024] = {0};
  StaticJsonDocument<1024> doc;
  //overrite the last strings 
  // wake up the big sensors
  novaSoft.listen();
  sds.wakeup();
  delay(800);

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
  // wdt_enable( WDT_PERIOD_8KCLK_gc);
  String timeStamp="";
  timeStamp=(getDateNow(rtc)+"-"+getTimeNow(rtc));
  // wdt_disable();
  
  readData(doc);// first read from the main sensors

  #ifdef DEBUG_MODE
  SerialMon.println(millis());
  #endif
  JsonArray data = doc.createNestedArray("soil");
  data.add(SMTtemp);
  data.add(SMTmois);
  doc["timestamp"] = timeStamp;

  serializeJson(doc, payload);
  
  char dataToSave[FILE_LINE_LENGTH+1];
  strcpy(dataToSave, payload); // should copy the first 512 bytes + 0

  formatData(dataToSave);
  digitalWrite(SD_CARD_LED,HIGH);
  saveData(dataToSave, dataFile);
  #ifdef DEBUG_MODE
  SerialMon.println(millis());
  #endif
  // setupgsm();
  connectnet();

  // how to change OLD_DATA_AVAILABLE to true?
  if (sendData(payload, NEW_DATA)) {
    if (OLD_DATA_AVAILABLE) {
      if (timeLeft() > DATA_SEND_TIME) {
        memset(payload, 0, 1024);
        if (readOldData(payload, dataFile)) {
          sendData(payload, OLD_DATA);
        } /*else {
          // in case there's no data or an error in reading, 
          // do nothing.
        }*/
        // timeLeft = PERIOD - (millis() - startTime);
      }
    }
  }

  digitalWrite(SD_CARD_LED,LOW);
  http.stop();
  #ifdef DEBUG_MODE
  SerialMon.println(F("Server disconnected"));
  #endif
  modem.gprsDisconnect();
  #ifdef DEBUG_MODE
  SerialMon.println(F("GPRS disconnected"));
  #endif

  softrst++;
  if (softrst < 36){
  while (timeLeft() > 0);
  }else{
    // watchdogEnable();
    while(1);
    }
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

  //communication variables
  // const char server[]   = "35.226.209.188";
  const char server[]   = "137.63.184.136";
  const int  port       = 8000;//port http

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

  #ifdef DEBUG_MODE
  SerialMon.println("Wait...");
  #endif
  //modem.setBaud(9600);
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  delay(6000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  #ifdef DEBUG_MODE
  SerialMon.println("Initializing modem...");
  #endif
  modem.restart();
   //modem.init();
  #ifdef DEBUG_MODE
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  #endif
}


 void connectnet(){
  setupgsm();
  #ifdef DEBUG_MODE
  SerialMon.print("Waiting for network...");
  #endif
  if (!modem.waitForNetwork()) {
    #ifdef DEBUG_MODE
    SerialMon.println(" fail");
    #endif
    // watchdogEnable(); 
    while(true);  
  }
  #ifdef DEBUG_MODE
  SerialMon.println(" success");
  #endif
  if (modem.isNetworkConnected()) { 
    #ifdef DEBUG_MODE
    SerialMon.println("Network connected");
    #endif
  }
  #ifdef DEBUG_MODE
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  #endif
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    #ifdef DEBUG_MODE
    SerialMon.println(" fail");
    #endif
    // watchdogEnable(); 
      while(true);
  }
  #ifdef DEBUG_MODE
  SerialMon.println(" success");
  #endif

  #ifdef DEBUG_MODE
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected"); 
  }
  #endif
  
  delay(5000);
}

// --- PLEASE COMMENT THE CODE NEXT TIME!! --- //

/*void watchdogEnable()
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
}*/

int sendData(char* postData, uint8_t age) {
  char resource[]="/api_v1/general/";//endpoint will be hard written here
  char contentType[] ="application/json";

  // watchdogEnable(); // In case connection to server fails
  #ifdef DEBUG_MODE
  SerialMon.print(F("Performing HTTP POST request... "));
  #endif
  //http.connectionKeepAlive();  // Currently, this is needed for HTTPS
  //int err = http.get(resource);
  int err = http.post(resource,contentType,postData);//even if they are char arrays
  if (err != 0) {
    #ifdef DEBUG_MODE
    SerialMon.println(F("failed to connect"));
    #endif
    while(true); // To be caught by WDT
  }
  // wdt_disable();
  #ifdef DEBUG_MODE
  Serial.println("**** starting loop ****");
  #endif
  int status = http.responseStatusCode();
  #ifdef DEBUG_MODE
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  #endif
  if ((status != 200) && (status != -3)) {
    return 0;
  }

  // watchdogEnable(); 
  if (status == -3) {
    markData(age, dataFile);
    digitalWrite(SD_CARD_LED,LOW);
    http.stop();
    #ifdef DEBUG_MODE
    SerialMon.println(F("Server disconnected"));
    #endif
    modem.gprsDisconnect();
    #ifdef DEBUG_MODE
    SerialMon.println(F("GPRS disconnected"));
    #endif
    while(1) {}
  }
  #ifdef DEBUG_MODE
  SerialMon.println(F("Response Headers:"));
  #endif
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    //SerialMon.println("    " + headerName + " : " + headerValue);
  }
  // int length = http.contentLength();
  // if (length >= 0) {
  //   //SerialMon.print(F("Content length is: "));
  //   //SerialMon.println(length);
  // }
  // if (http.isResponseChunked()) {
  //   //SerialMon.println(F("The response is chunked"));
  // }

  // String body = http.responseBody();
  // //SerialMon.println(F("Response:"));
  // //SerialMon.println(body);

  // //SerialMon.print(F("Body length is: "));
  // //SerialMon.println(body.length());
  // // wdt_disable();
  
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

String getDateNow(DS3231 clock) {
  // if it was 2095^, we'd need to plan for a century roll over.
  bool century;
  String out = String(clock.getDate(), DEC);
  out += '.';
  out += String(clock.getMonth(century), DEC);
  out += '.';
  out += String(clock.getYear(), DEC);
  return out; // dd.mm.yyyy
}

String getTimeNow(DS3231 clock) {
  bool h12; // 12-hour/~24-hour flag
  bool PM; // PM/~AM flag
  String out = String(clock.getHour(h12, PM), DEC) + ':';
  out += String(clock.getMinute(), DEC) + ':';
  out += String(clock.getSecond(), DEC);
  return out; // HH:MM:SS
}

unsigned long timeLeft() {
  return (timeElapsed() < PERIOD) ? PERIOD - timeElapsed() : 0;
}

unsigned long timeElapsed() {
  return (millis() - startTime);
}

void configureCLKRTC() {
  // configure CLK_RTC
  RTC.CLKSEL |= RTC_CLKSEL0_bm; // INT32K OSCULP32K internal clock
}

void rtcwdt() {
  configureCLKRTC();
  while(RTC.STATUS & RTC_CTRLABUSY_bm); // wait for status to not be busy
  // set RTC.CMP ==> 300 mins to 1001 0110 0000 0000
  RTC.CMPL &= 0x00;
  RTC.CMPH &= 0x96 ;

  // Enable interrupts, RTC.INTCTRL
  RTC.INTCTRL |= RTC_CMP_bm;

  // Configure prescaler and enable bit
  while(RTC.STATUS & RTC_CTRLABUSY_bm); // wait for CTRL status to not be busy
  RTC.CTRLA |= (0x08<<3); // set 256 PRESCALER
  RTC.CTRLA |= RTC_RTCEN_bm; // set RTC enable
}

ISR(RTC_CNT_vect) {
  RTC.INTFLAGS |= RTC_CMP_bm;
  
  // start watchdog
  RSTCTRL.RSTFR |= RSTCTRL_WDRF_bm ;
  wdt_enable(WDT_PERIOD_8KCLK_gc);
}
