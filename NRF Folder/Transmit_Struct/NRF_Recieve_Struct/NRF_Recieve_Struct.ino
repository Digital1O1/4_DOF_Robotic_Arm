// UNO Reciever Code
// Reference : https://stackoverflow.com/questions/53717218/using-nrf24l01-between-2-arduinos-to-send-multiple-sensor-values
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "U8g2lib.h"

RF24 radio(7, 8);                           // CE, CSN
const byte address[6] = "00001";            // Address the the module
char val1_buffer[20], val2_buffer[20], val3_buffer[20], val4_buffer[20], val5_buffer[20];

U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

struct data_Struct
{
  float val1, val2, val3;
  int val4, val5;
};

void setup()
{
  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);

  radio.startListening();

  // OLED Stuff
  //OLED.setI2CAddress(0x3C * 2);
  //OLED.begin();
  //OLED.setFont(u8g2_font_7x13B_mr);
}

void loop()
{
  data_Struct recieved_Data;
  if (radio.available())
  {
    radio.read(&recieved_Data, sizeof(recieved_Data));

    Serial.print("Roll : "); Serial.print(recieved_Data.val1);
    Serial.print("\tYaw : "); Serial.print(recieved_Data.val2);
    Serial.print("\tRoll : "); Serial.print(recieved_Data.val3);
    Serial.print("\tMap X : "); Serial.print(recieved_Data.val4);
    Serial.print("\tMap Y : "); Serial.print(recieved_Data.val5);
    Serial.print("\tSize of Data Packet : "); Serial.println(sizeof(recieved_Data));

    /*
      OLED.clearBuffer();
      OLED.drawStr(25, 10, "VALUES RECEIVED");
      dtostrf(recieved_Data.val1, 5, 3, val1_buffer);
      dtostrf(recieved_Data.val2, 5, 3, val2_buffer);
      dtostrf(recieved_Data.val3, 5, 3, val3_buffer);
      dtostrf(recieved_Data.val4, 5, 3, val4_buffer);

      OLED.drawStr(5, 10, "Pitch : ");
      OLED.drawStr(80, 10, val1_buffer);

      OLED.drawStr(5, 22, "Yaw : ");
      OLED.drawStr(80, 22, val2_buffer);

      OLED.drawStr(5, 35, "Roll : ");
      OLED.drawStr(80, 35, val3_buffer);

      OLED.drawStr(5, 52, "Map X : ");
      OLED.drawStr(80, 52, val4_buffer);

      OLED.drawStr(5, 64, "Map Y : ");
      OLED.drawStr(80, 64, val5_buffer);
      OLED.sendBuffer();
    */
  }

}
