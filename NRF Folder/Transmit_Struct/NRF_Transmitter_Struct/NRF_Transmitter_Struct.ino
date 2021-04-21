// TEENSY Transmitter
// Reference link : https://forum.arduino.cc/index.php?topic=421081
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "U8g2lib.h"

RF24 radio(7, 8);                         // CE, CSN Pins
const byte address[6] = "00001";          // Address the the module
float send_Data1, send_Data2 = 0;
char val1_buffer[20], val2_buffer[20], val3_buffer[20], val4_buffer[20], val5_buffer[20];
bool send_flag;
struct data_Struct
{
  float val1 = random(10, 250) / 10.0;
  float val2 = random(10, 60) / 2.3;
  float val3 = random(10, 60) / 2.3;
  int   val4 = random(1, 4);
  int   val5 = random(0, 40);

};
U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup()
{
  Serial.begin(115200);

  // OLED Stuff
  OLED.setI2CAddress(0x3D * 2);
  OLED.begin();
  OLED.setFont(u8g2_font_7x13B_mr);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);
  radio.stopListening();
}
void loop()
{
  //Sending two values
  data_Struct sendData;
  send_flag = radio.write(&sendData, sizeof(sendData));
  if (send_flag == true)
  {
    Serial.print("Value 1 : "); Serial.print(sendData.val1);
    Serial.print("\tValue 2 : "); Serial.print(sendData.val2);
    Serial.print("\tValue 3 : "); Serial.print(sendData.val3);
    Serial.print("\tValue 4 : "); Serial.print(sendData.val4);
    Serial.print("\tValue 5 : "); Serial.print(sendData.val5);
    Serial.print("\tData Packet Size : "); Serial.println(sizeof(sendData));
    //Serial.print("\t\tResult : "); Serial.println(send_flag);

    OLED.clearBuffer();
    OLED.drawStr(25, 10, "VALUES SENT");
    dtostrf(sendData.val1, 5, 3, val1_buffer);
    dtostrf(sendData.val2, 5, 3, val2_buffer);
    dtostrf(sendData.val3, 5, 3, val3_buffer);
    dtostrf(sendData.val4, 5, 3, val4_buffer);
    dtostrf(sendData.val5, 5, 3, val5_buffer);

    OLED.drawStr(45, 40, val1_buffer);
    OLED.drawStr(45, 50, val2_buffer);
    OLED.drawStr(45, 60, val1_buffer);
    OLED.drawStr(45, 70, val2_buffer);
    OLED.drawStr(45, 80, val2_buffer);

    OLED.sendBuffer();
  }

  delay(100);

}
