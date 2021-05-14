// Slave 0

#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h>

#define CE_PIN   7
#define CSN_PIN 8

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

float dataReceived[100]; // this must match dataToSend in the TX
int ackData[2] = {109, -4000}; // the two values to be sent to the master
bool newData = false;

//==============

void setup() {

  Serial.begin(115200);

  Serial.println("SimpleRxAckPayload Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.enableAckPayload();
  radio.startListening();
  radio.writeAckPayload(1, &ackData, sizeof(ackData)); // pre-load data
  radio.setPALevel(RF24_PA_MIN);

}

//==========

void loop() {
  getData();
  showData();
}

//============

void getData()
{
  if ( radio.available() )
  {
    radio.read( &dataReceived, sizeof(dataReceived) );
    //updateReplyData();
    newData = true;
  }
}

//================

void showData() {
  if (newData == true)
  {
    Serial.println("-------------------");
    Serial.println("Nano Data received ");
    //Serial.print(dataReceived[0]);
    for (int i = 0; i < 5; i++)
    {
      Serial.print(dataReceived[i]);
      Serial.print("\t");
    }
    Serial.println();
    //Serial.print("\nackPayload sent ");
    //Serial.print(ackData[0]);
    //Serial.print(", ");
    //Serial.println(ackData[1]);
    newData = false;
  }
  else
  {
    //Serial.println("Waiting for data.....");
  }
}

//================

void updateReplyData()
{
  ackData[0] -= 1;
  ackData[1] -= 1;
  if (ackData[0] < 100)
  {
    ackData[0] = 109;
  }
  if (ackData[1] < -4009)
  {
    ackData[1] = -4000;
  }
  radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
}
