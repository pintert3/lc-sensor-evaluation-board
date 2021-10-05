#include <SD.h>

const int LED = 4;
int oldDataAvailable = 1;
const uint8_t FILE_LINE_LENGTH = 9;
const uint8_t NEW_DATA = 1;
const uint8_t OLD_DATA = 0;

void setup() {
  pinMode(53, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  SD.begin(53);
  delay(8000); // 5 sec to start minicom
  Serial.println("..--=====STARTED=====--..");

  // tests
  char output[512] = {};
  int passed = 0;
  passed += test_read_old_data(output);
  passed += test_mark_data(NEW_DATA);
  passed += test_mark_data(OLD_DATA);
  Serial.println("............................");
  Serial.print(String("Passed: ")+String(passed));
  Serial.println(" out of 3 tests");
  Serial.println("...........................\n");
}

void loop() {
}

int test_mark_data(uint8_t age) {
  int check;
  if (age) {
    check = markData(age, String("mark_new.txt"));
  } else {
    check = markData(age, String("mark_old.txt"));
  }

  if (!check) {
    Serial.println("Marked");
    Serial.print("Test Mark");
    Serial.print(String((age) ? "New data" : "Old data"));
    Serial.println(".....SUCCESS");
    return 1;
  } else {
    switch (check) {
      case 1:
        Serial.println("No data to mark found.");
        break;
      case 2:
        Serial.println("Error, data is not well formatted.");
        break;
      case 3: 
        Serial.println("Error: File couldn't be opened.");
        break;
      case 4:
        Serial.println("Error: no file with that name found.");
        break;
      default:
        Serial.println("Unexpected error.");
        break;
    }
    Serial.print("Test Mark");
    Serial.print(String((age) ? "New data" : "Old data"));
    Serial.println(".....FAILED");
    return 0;
  }
}

int test_read_old_data(char output[]) {
  if (readOldData(output, String("old_data.txt"))) {
    Serial.println("------------------\n");
    Serial.print("Output:");
    Serial.println(String(output));
    Serial.println("------------------\n");
    if (String(output) == String("ghijkl")) {
      Serial.println("Test Read_Old_Data.....SUCCESS");
      return 1;
    } else {
      Serial.println("Wrong String returned");
      Serial.println("Test Read_Old_Data.....FAILED");
      return 0;
    }
  } else {
    Serial.println("Failed to read old data.");
    Serial.println("Test Read_Old_Data.....FAILED");
    return 0;
  }
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


int markData(uint8_t age, String filename) {
  int output_code = 1;
  char nextChar;
  if(SD.exists(filename)){
    // open data file
    File sensorData = SD.open(filename, O_RDWR);
    if (sensorData){
      Serial.println("Marking...");
      if (age == NEW_DATA) {
        // seek back to start of last line and add '?'
        // nextChar=sensorData.peek();
        // while (nextChar != -1) {
        //   sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2);
        //   nextChar = sensorData.peek();
        // }
        sensorData.seek(sensorData.size());
        Serial.print("end found: ");
        Serial.println(nextChar);
        sensorData.seek(sensorData.position()+(FILE_LINE_LENGTH*-1) - 2); 
        sensorData.write('?');
        output_code = 0;
      } else { // Marking OLD_DATA
        // sensorData.seek(0);
        nextChar = sensorData.peek();
        while (nextChar != -1) { // not expecting zero(0)
          if (nextChar == '!') {
            blinkled(LED);
            Serial.println(nextChar);
            sensorData.write('?');
            Serial.println("Done Marking."); // DEBUG
            output_code = 0;
            break;
          } else if (nextChar == '?') {
            sensorData.seek(sensorData.position()+FILE_LINE_LENGTH+2); // Skip the ? and the \n
            nextChar = sensorData.peek();
            Serial.println(nextChar);
          } else {
            Serial.println("Error, data is not well formatted");
            output_code = 2; // means error with data format(arrangement)
          }
        }
      }
      sensorData.close();
    }else{
      Serial.println("Error: File couldn't be opened");
      output_code = 3;
    }
  } else {
    Serial.println("Error: no file with that name found.");
    output_code = 4;
  }
  return output_code;
}
