//Reciever Code

#include <SPI.h> // for SPI communication
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001"; // the address the the module

void setup() 
{
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

}

void loop() 
{
  if (radio.available()) { // if nrf has any incoming data
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.print("Recieved : ");Serial.println(text);
    delay(5);
  }
}
