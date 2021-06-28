#include <PMserial.h>

// check if this is right, Apparently there's Serial1,Serial2, etc.
// I don't know if doing this has any effect on our serial print
// console.
SerialPM pmsa(PMSA003, Serial); 

void setup() {
    Serial.begin(9600);
    pmsa.init();
}

void loop() {
    // MPIIMA: you might have to make an array of these :D :D
    // Reason why I didn't?: Guys here called me to do some other
    // annoying thing, and I was trying to get the code ready as
    // early as possible, yet I have to first read how to make C
    // arrays before I do that. In case you can't, I'll have to do
    // it around 8 or 9PM maybe
    pmsa.read();
    if (pmsa) {
        // print results
        Serial.print(F("PM1.0 "));
        Serial.print(pmsa.pm01);
        Serial.print(F(", "));
        Serial.print(F("PM2.5 "));
        Serial.print(pmsa.pm25);
        Serial.print(F(", "));
        Serial.print(F("PM10 "));
        Serial.print(pmsa.pm10);
        Serial.println(F(" [ug/m3]"));
    
        if (pmsa.has_number_concentration())
        {
            Serial.print(F("N0.3 "));
            Serial.print(pmsa.n0p3);
            Serial.print(F(", "));
            Serial.print(F("N0.5 "));
            Serial.print(pmsa.n0p5);
            Serial.print(F(", "));
            Serial.print(F("N1.0 "));
            Serial.print(pmsa.n1p0);
            Serial.print(F(", "));
            Serial.print(F("N2.5 "));
            Serial.print(pmsa.n2p5);
            Serial.print(F(", "));
            Serial.print(F("N5.0 "));
            Serial.print(pmsa.n5p0);
            Serial.print(F(", "));
            Serial.print(F("N10 "));
            Serial.print(pmsa.n10p0);
            Serial.println(F(" [#/100cc]"));
        }
    
        if (pmsa.has_temperature_humidity() || pmsa.has_formaldehyde())
        {
            Serial.print(pmsa.temp, 1);
            Serial.print(F(" °C"));
            Serial.print(F(", "));
            Serial.print(pmsa.rhum, 1);
            Serial.print(F(" %rh"));
            Serial.print(F(", "));
            Serial.print(pmsa.hcho, 2);
            Serial.println(F(" mg/m3 HCHO"));
        }
    } else {
        switch (pmsa.status)
        {
        case pmsa.OK: // should never come here
              break;     // included to compile without warnings
        case pmsa.ERROR_TIMEOUT:
              Serial.println(F(PMS_ERROR_TIMEOUT));
              break;
        case pmsa.ERROR_MSG_UNKNOWN:
              Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
              break;
        case pmsa.ERROR_MSG_HEADER:
              Serial.println(F(PMS_ERROR_MSG_HEADER));
              break;
        case pmsa.ERROR_MSG_BODY:
              Serial.println(F(PMS_ERROR_MSG_BODY));
              break;
        case pmsa.ERROR_MSG_START:
              Serial.println(F(PMS_ERROR_MSG_START));
              break;
        case pmsa.ERROR_MSG_LENGTH:
              Serial.println(F(PMS_ERROR_MSG_LENGTH));
              break;
        case pmsa.ERROR_MSG_CKSUM:
              Serial.println(F(PMS_ERROR_MSG_CKSUM));
              break;
        case pmsa.ERROR_PMS_TYPE:
              Serial.println(F(PMS_ERROR_PMS_TYPE));
              break;
        }

    }
}
