// Transmitter code

#include <SPI.h> // for SPI communication
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001"; // the address the the module

void setup() 
{
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  
}

void loop() 
{
  const char text[] = "Hello World"; // you can customize this text to your wish
  radio.write(&text, sizeof(text));
  Serial.println(text);
}
