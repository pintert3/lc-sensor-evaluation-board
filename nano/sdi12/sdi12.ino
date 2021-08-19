
/*
  ======= FLOW OF PROGRAM ==========
 ----:SETUP:-----------
 - set data pin of sdi 12
 - set storage variable for pmsa data
 - setup serial pins for nano-PMSA communication
 - setup serial communication for nano-PMSA
 -----:END SETUP--------

 ----:LOOP:-----------
 1. check if SEN5 logger is requesting data
 2. If true: 
  2a. parse sdi12 command
  2b. collect data from PMSA if requested
  2c. respond to sdi12 command
 ----:END LOOP:---------
  ===================================
*/


#include <SDI12.h>
#include <PMserial.h>

#define DATA_PIN 7   /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1 /*!< The sensor power pin (or -1 if not switching power) */

char sensorAddress = '?';
int  state         = 1;

#define WAIT 0

SerialPM pmSensor(PMSA003, Serial);
// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12 slaveSDI12(DATA_PIN);


void pollSensor(uint16_t* measurementValues) {
  // Gets data from PMSA003 
  // Only reads particulate matter values, no NC values
  pmSensor.read();
  if (pmSensor) {
    for (int i = 0; i < 3; i++) {
      measurementValues[i] = pmSensor.pm[i];
    }
  }
}

void parseSdi12Cmd(String command, String dValues, uint16_t* measurementValues) {
  /* Ingests a command from an SDI-12 master, sends the applicable response, and
   * (when applicable) sets a flag to initiate a measurement
   */

  // First char of command is always a '?' for address query.
  // Do nothing if this command is misaddressed
  if (command.charAt(0) != '?') { return; }

  // If execution reaches this point, the slave should respond with something in
  // the form:   <address><responseStr><Carriage Return><New Line> (SEN 5)
  // The following if-switch-case block determines what to put into <responseStr>,
  // and the full response will be constructed afterward. For '?!' (address query)
  // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
  String responseStr = "";
  if (command.length() > 1) {
    switch (command.charAt(1)) {
      case 'I':
        // Identify command
        // Slave should respond with ID message: 2-char SDI-12 version + 8-char
        // company name + 6-char sensor model + 3-char sensor version + 0-13 char S/N
        responseStr = "14PLANTOWE0000102.5001";  // PLANTOWER --!! Needs S/N
        break;
      
      case 'R':
        // Send data command
        // Slave should respond with a String of values
        // Values to be returned must be split into Strings of 35 characters or fewer
        // (75 or fewer for concurrent).  The number following "D" in the SDI-12 command
        // specifies which String to send

        /* 1. get data from PMSA
         * 2. return the collected data
        */

        pollSensor(measurementValues);
        responseStr = dValues; // TAHMO: should be a single value, cause only aR0 used
        break;
      
      default:
        // Mostly for debugging; send back UNKN if unexpected command received
        responseStr = "UNKN";
        break;
    }
  }

  // Issue the response speficied in the switch-case structure above.
  slaveSDI12.sendResponse(String(sensorAddress) + responseStr + "\r\n");
}


void formatOutputSDI(uint16_t* measurementValues, String dValues, unsigned int maxChar) {
  /* Ingests an array of floats and produces Strings in SDI-12 output format */

  dValues = "";
  int j   = 0;

  // upper limit on i should be number of elements in measurementValues
  for (int i = 0; i < 3; i++) {
    // Read float value "i" as a String with 6 deceimal digits
    // TAHMO: Should read *integer* value instead, as string with _5_ decimal digits
    // (NOTE: SDI-12 specifies max of 7 digits per value; we can only use 6
    //  decimal place precision if integer part is one digit)

    // need to check the length of pmsa data, and send it as integer, not float
    String valStr = String(measurementValues[i]); // TAHMO: change to pmsa data
    // Explictly add implied + sign if non-negative
    if (valStr.charAt(0) != '-') { valStr = '+' + valStr; }
    // Append dValues[j] if it will not exceed 35 (aM!) or 75 (aC!) characters
    if (dValues.length() + valStr.length() < maxChar) {
      dValues += valStr;
    }
    // Start a new dValues "line" if appending would exceed 35/75 characters
    else {
      // should not be possible
    }
  }

}


void setup() {
  slaveSDI12.begin();
  delay(500);
  slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message
  pmSensor.init();
}

void loop() {
  static uint16_t measurementValues[3];  // 3 ints to hold pmsa data
  
  // TAHMO: Since we use only ?R0, this should only save space for one value (dvalues[1])
  static String dValues;  // 10 String objects to hold the responses to aD0!-aD9! commands
  static String commandReceived = "";  // String object to hold the incoming command


  // If a byte is available, an SDI message is queued up. Read in the entire message
  // before proceding.  It may be more robust to add a single character per loop()
  // iteration to a static char buffer; however, the SDI-12 spec requires a precise
  // response time, and this method is invariant to the remaining loop() contents.
  int avail = slaveSDI12.available();
  if (avail < 0) {
    slaveSDI12.clearBuffer();
  }  // Buffer is full; clear
  else if (avail > 0) {
    for (int a = 0; a < avail; a++) {
      char charReceived = slaveSDI12.read();
      // Character '!' indicates the end of an SDI-12 command; if the current
      // character is '!', stop listening and respond to the command
      if (charReceived == '!') {
        // Command string is completed; do something with it
        parseSdi12Cmd(commandReceived, dValues, measurementValues);
        // Clear command string to reset for next command
        commandReceived = "";
        // '!' should be the last available character anyway, but exit the "for" loop
        // just in case there are any stray characters
        slaveSDI12.clearBuffer();
        // eliminate the chance of getting anything else after the '!'
        slaveSDI12.forceHold();
        break;
      }
      // If the current character is anything but '!', it is part of the command
      // string.  Append the commandReceived String object.
      else {
        // Append command string with new character
        commandReceived += String(charReceived);
      }
    }
  }

}
