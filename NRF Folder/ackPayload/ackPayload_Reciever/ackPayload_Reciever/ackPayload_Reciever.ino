// HandController - RECIEVER

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

#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#define CE_PIN 7
#define CSN_PIN 8

// NOTE: the "LL" at the end of the constant is "LongLong" type

const uint64_t deviceID = 0xE8E8F0F0E1LL; // Define the ID for this slave

int valChange = 1;

RF24 radio(CE_PIN, CSN_PIN);

int dataReceived[2];
int ackData[2] = {12, 0};
byte ackLen = 2;

void setup()
{

    Serial.begin(115200);
    delay(100);
    Serial.println("Hand Controller Starting");
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, deviceID);
    radio.enableAckPayload();
    radio.writeAckPayload(1, ackData, ackLen);
    radio.startListening();
}

void loop()
{

    if (radio.available())
    {
        radio.read(dataReceived, sizeof(dataReceived));
        Serial.print("Data received Number0 ");
        Serial.print(dataReceived[0]);
        Serial.print(" Number1 ");
        Serial.println(dataReceived[1]);
        radio.writeAckPayload(1, ackData, ackLen);
        ackData[0] += valChange; // this just increments so you can see that new data is being sent
    }
}
