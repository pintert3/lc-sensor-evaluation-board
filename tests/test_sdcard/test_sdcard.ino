#include <SD.h>

const int LED = 4;
int oldDataAvailable = 1;
const uint8_t FILE_LINE_LENGTH = 9;
const uint8_t NEW_DATA = 1;

void setup() {
  pinMode(53, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  SD.begin(53);
  delay(5000); // 5 sec to start minicom
  Serial.println("..--=====STARTED=====--..");
}

void loop() {
  char output[512] = {};
  //char* payload;
  if (readOldData(output, String("neuer.txt"))) {
    Serial.println("------------------\n");
    Serial.print("Output:");
    Serial.println(String(output));
    //Serial.print("Payload:");
    //Serial.println(String(payload));
    Serial.println("------------------\n");
    markData(0, String("neuer.txt"));
  }
  //Serial.println("Read data:");
  //Serial.println(output);
  delay(4000);
}

int readOldData(char* output, String filename){
  // Should return 1 if old data is found
  // else, it should return 0

  char nextChar = -1;

  if(SD.exists(filename)){ 
    char buffer[FILE_LINE_LENGTH];

    // open data file if opening went well
    File sensorData = SD.open(filename, FILE_READ);
    if (sensorData) {
      // TODO: Read data from file
      nextChar = sensorData.peek();
      while (nextChar >= 0) {
        if (nextChar == '!') {
          Serial.println("UnMarked."); // DEBUG
          blinkled(LED);
          sensorData.seek(sensorData.position()+1);
          sensorData.read(buffer, FILE_LINE_LENGTH);
          prepare_payload(buffer, output);
          delay(500);
          Serial.println("Unprocessed:");
          delay(500);
          Serial.println(String(buffer));
          delay(2000);
          break;
        } else if (nextChar == '?') {
          sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2);
          nextChar = sensorData.peek(); // DEBUG
          Serial.print("Next one: ");
          Serial.println(nextChar);
          delay(1000);
        } else {
          Serial.println("Error, data is not well formatted");
        }
      }

      sensorData.close(); // close the file

      if (nextChar < 0) {
        oldDataAvailable = 0;
        return 0;
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }else{
    //Serial.println("Error writing to file !"); place indicator led
    Serial.println("No file named: ");
    Serial.println(filename);
    return 0;
  }

}

void blinkled(int led) {
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
}

void prepare_payload(char* unfiltered_data, char* output) {
  // loop forwards the array of length FILE_LINE_LENGTH
  // until find \t character.
  // send that to payload
  int i;
  for (i = 0; ((i < FILE_LINE_LENGTH) && (unfiltered_data[i] != '\t')); i++) {
    output[i] = unfiltered_data[i];
  }
}


void markData(uint8_t age, String filename) {
  char nextChar;
  if(SD.exists(filename)){
    // open data file
    File sensorData = SD.open(filename, O_RDWR);
    if (sensorData){
      Serial.println("Marking...");
      if (age == NEW_DATA) {
        // seek back to start of last line and add '?'
        sensorData.seek(sensorData.position()+(FILE_LINE_LENGTH*-1) - 2); // need to overwrite 
        nextChar = sensorData.peek();
        Serial.println(nextChar);
        sensorData.write('?');
      } else {
        sensorData.seek(0);
        nextChar = sensorData.peek();
        while (nextChar > 0) { // not expecting zero(0)
          if (nextChar == '!') {
            blinkled(LED);
            Serial.println(nextChar);
            sensorData.write('?');
            Serial.println("Done Marking."); // DEBUG
            break;
          } else if (nextChar == '?') {
            sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2); // Skip the ? and the \n
            nextChar = sensorData.peek();
            Serial.println(nextChar);
          } else {
            Serial.println("Error, data is not well formatted");
          }
        }
      }
      sensorData.close();
    }else{
      Serial.println("Error: no file with that name found.");
    }
  }
}
