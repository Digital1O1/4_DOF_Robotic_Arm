#include <SPI.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "U8g2lib.h"
Servo servo_X;                            
Servo servo_Y;                            

float data[5];
RF24 radio(7, 8);
const byte address[6] = "00001";          
char pitch_buffer[20], roll_buffer[20], yaw_buffer[20], mapX_buffer[20], mapY_buffer[20];

U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup(void)
{
  // Serial Stuff
  Serial.begin(115200);

  // OLED Stuff
  OLED.setI2CAddress(0x3C * 2);
  OLED.begin();
  OLED.setFont(u8g2_font_7x13B_mr);

  // RF Stuff
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);
  radio.startListening();

  // Servo Stuff
  servo_X.attach(5);  // attaches the servo on pin 9 to the servo object
  servo_Y.attach(6);  // attaches the servo on pin 9 to the servo object

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
    Serial.println();
    OLED.clearBuffer();
    //OLED.drawStr(15, 10, "VALUES RECIEVED");
    dtostrf(data[0], 5, 3, pitch_buffer);
    dtostrf(data[1], 5, 3, yaw_buffer);
    dtostrf(data[2], 5, 3, roll_buffer);
    dtostrf(data[3], 5, 3, mapX_buffer);
    dtostrf(data[4], 5, 3, mapY_buffer  );

    OLED.drawStr(5, 10, "Pitch : ");
    OLED.drawStr(80, 10, pitch_buffer);

    OLED.drawStr(5, 22, "Roll : ");
    OLED.drawStr(80, 22, roll_buffer);

    OLED.drawStr(5, 35, "Yaw : ");
    OLED.drawStr(80, 35, yaw_buffer);

    OLED.drawStr(5, 52, "Map_X : ");
    OLED.drawStr(80, 52, mapX_buffer);

    OLED.drawStr(5, 64, "Map_Y : ");
    OLED.drawStr(80, 64, mapY_buffer);

    OLED.sendBuffer();

  }
}
