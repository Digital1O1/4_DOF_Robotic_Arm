#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "U8g2lib.h"
float data[5];

RF24 radio(7, 8);
const byte address[6] = "00001";          // Address the the module
char val1_buffer[20], val2_buffer[20], val3_buffer[20], val4_buffer[20], val5_buffer[20];

U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup(void)
{
  Serial.begin(115200);

  // OLED Stuff
  OLED.setI2CAddress(0x3C * 2);
  OLED.begin();
  OLED.setFont(u8g2_font_7x13B_mr);

  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);
  radio.startListening();
}

void loop(void)
{
  if ( radio.available() )
  {

    radio.read(data, sizeof(data));
    for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++)
    {
      Serial.print("Data[");
      Serial.print(i);
      Serial.print("] : ");
      Serial.print(data[i]);
      Serial.print("\t\t");
    }
    Serial.print("Packet size : "); Serial.println(sizeof(data));
    OLED.clearBuffer();
    //OLED.drawStr(15, 10, "VALUES RECIEVED");
    dtostrf(data[0], 5, 3, val1_buffer);
    dtostrf(data[1], 5, 3, val2_buffer);
    dtostrf(data[2], 5, 3, val3_buffer);
    dtostrf(data[3], 5, 3, val4_buffer);
    dtostrf(data[4], 5, 3, val5_buffer);

    OLED.drawStr(5, 10, "Pitch : ");
    OLED.drawStr(80, 10, val1_buffer);

    OLED.drawStr(5, 22, "Yaw : ");
    OLED.drawStr(80, 22, val2_buffer);

    OLED.drawStr(5, 35, "Roll : ");
    OLED.drawStr(80, 35, val3_buffer);

    OLED.drawStr(5, 52, "Map_X : ");
    OLED.drawStr(80, 52, val4_buffer);

    OLED.drawStr(5, 64, "Map_Y : ");
    OLED.drawStr(80, 64, val5_buffer);

    OLED.sendBuffer();

  }
}
