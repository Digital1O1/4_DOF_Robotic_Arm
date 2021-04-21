#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

// TrackControl - TRANSMITTER

 // http://tmrh20.github.io/RF24/

 //~ - CONNECTIONS: nRF24L01 Modules See:
 //~ http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
 //~ 1 - GND
 //~ 2 - VCC 3.3V !!! NOT 5V
 //~ 3 - CE to Arduino pin 9
 //~ 4 - CSN to Arduino pin 10
 //~ 5 - SCK to Arduino pin 13
 //~ 6 - MOSI to Arduino pin 11
 //~ 7 - MISO to Arduino pin 12
 //~ 8 - UNUSED


#define CE_PIN   7
#define CSN_PIN 8

// NOTE: the "LL" at the end of the constant is "LongLong" type
// These are the IDs of each of the slaves
const uint64_t slaveID[2] = {0xE8E8F0F0E1LL, 0xE8E8F0F0E2LL} ;

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

int dataToSend[2];

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 500;
int txVal = 0;
int ackMessg[4];
byte ackMessgLen = 2;


void setup() {

    Serial.begin(115200);
    Serial.println("Track Control Starting");
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.enableAckPayload();
    radio.setRetries(3,5); // delay, count
}

//====================

void loop() {

    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {

    radio.openWritingPipe(slaveID[0]);  // calls the first slave
                                        // there could be a FOR loop to call several slaves in turn
    dataToSend[0] = txVal;              // this gets incremented so you can see that new data is being sent
    txVal += 1;
    dataToSend[1] = txVal;
    txVal += 1;
    bool rslt;
    rslt = radio.write( dataToSend, sizeof(dataToSend) );
    Serial.print("\nRSLT (1 = success) ");
    Serial.println(rslt);
    Serial.print("Data Sent ");
    Serial.print(dataToSend[0]);
    Serial.print("  ");
    Serial.println(dataToSend[1]);
    if ( radio.isAckPayloadAvailable() ) {
        radio.read(ackMessg,ackMessgLen);
        Serial.print("Acknowledge received: ");
        Serial.println(ackMessg[0]);
    }
    prevMillis = millis();
 }
}
