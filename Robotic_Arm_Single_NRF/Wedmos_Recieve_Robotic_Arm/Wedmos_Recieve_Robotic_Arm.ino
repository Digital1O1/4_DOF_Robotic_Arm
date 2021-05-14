// Slave 1

#include <SPI.h>
#include "U8g2lib.h"
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   16
#define CSN_PIN 15
U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'B'};
float sendData[5];

RF24 radio(CE_PIN, CSN_PIN);
float dataReceived[20]; // this must match dataToSend in the TX
int ackData[2] = {109, -4000}; // the two values to be sent to the master
char display_pitch[20], display_roll[20], display_yaw[20], display_map_X[20], display_map_Y[20];
float pitch, yaw, roll = 0;
int map_X, map_Y = 0;
bool newData = false;

//==============

void showData(int *counter);

void setup()
{

  Serial.begin(115200);

  Serial.println("SimpleRxAckPayload Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);

  radio.enableAckPayload();

  radio.startListening();

  radio.writeAckPayload(1, &ackData, sizeof(ackData)); // pre-load data

    // OLED Stuff
  OLED.setI2CAddress(0x3C * 2);
  OLED.begin();
  OLED.setFont(u8g2_font_7x13B_mr);
}

//==========

void loop() {
  getData();
  showData();
}

//============

void getData() {
  if ( radio.available() ) {
    radio.read( &dataReceived, sizeof(dataReceived) );
    //updateReplyData();
    newData = true;
  }
}

//================

void showData()
{
  if (newData == true)
  {
    Serial.println("--------------------------");
    Serial.println("Wedmos Data received ");
    /*
      for (int i = 0; i < 5; i++)
      {
      Serial.print(dataReceived[i]);
      Serial.print("\t");
      }
      Serial.println();
    */
    //Serial.print(dataReceived[0]);
    //Serial.print("\nackPayload sent ");
    //Serial.print(ackData[0]);
    //Serial.print(", ");
    //Serial.println(ackData[1]);
    pitch = dataReceived[0];
    yaw =dataReceived[1];
    roll=dataReceived[2];
    map_X=dataReceived[3];
    map_Y=dataReceived[4];

    Serial.print("Pitch : "); Serial.print(pitch);
    Serial.print("\tRoll : "); Serial.print(yaw);
    Serial.print("\tYaw: "); Serial.print(roll);
    Serial.print("\tMap_X : "); Serial.print(map_X);
    Serial.print("\tMax_Y : "); Serial.println(map_Y);

    // OLED STUFF
    dtostrf(roll, 2, 2, display_roll);
    dtostrf(pitch, 2, 2, display_pitch);
    dtostrf(yaw, 2, 2, display_yaw);
    dtostrf(map_X, 2, 2, display_map_X);
    dtostrf(map_Y, 2, 2, display_map_Y);

    OLED.drawStr(5, 10, "Pitch : ");
    OLED.drawStr(80, 10, display_pitch);

    OLED.drawStr(5, 22, "Roll : ");
    OLED.drawStr(80, 22, display_roll);

    OLED.drawStr(5, 35, "Yaw : ");
    OLED.drawStr(80, 35, display_yaw);

    OLED.drawStr(5, 52, "Map X : ");
    OLED.drawStr(80, 52, display_map_X);

    OLED.drawStr(5, 64, "Map Y : ");
    OLED.drawStr(80, 64, display_map_Y);

    OLED.sendBuffer();  // transfer internal memory to the display
    OLED.clearBuffer(); // clear the internal memory
    newData = false;
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
