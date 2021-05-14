#include "MPU9250.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   7
#define CSN_PIN 8

const byte numSlaves = 2;
// each slave needs a different address
const byte slaveAddress[numSlaves][5] =
{
  {'R', 'x', 'A', 'A', 'A'},
  {'R', 'x', 'A', 'A', 'B'}
};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI, 10);
int status;
float axb, ayb, azb, axs, ays, azs,
      aX, aY, aZ, gX, gY, gZ,
      gxb, gyb, gzb, mX, mY, mZ = 0;


int ackData[2] = { -1, -1}; // to hold the two values coming from the slave
float sendData[100];
bool radio_result;

void printData()
{
  // Print data
  Serial.print(sendData[0]); Serial.print('\t');
  Serial.print(sendData[1]); Serial.print('\t');
  Serial.print(sendData[2]); Serial.print('\t');
  Serial.print(sendData[3]); Serial.print('\t');
  Serial.print(sendData[4]); Serial.print('\t');
  Serial.print(sendData[5]); Serial.print('\t');
  Serial.print(sendData[6]); Serial.print('\t');
  Serial.print(sendData[7]); Serial.print('\t');
  Serial.println(sendData[8]);
}

void setup()
{
  // Establish Serial Communication
  Serial.begin(115200);
  Serial.println("-------------[ SERIAL COMMUNICATION ESTABLISHED ]-------------\n");
  Serial.println("-------------[ CHECKING IMU ]-------------\n");

  // Check if IMU is operational
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  else
  {
    Serial.println("-------------[ IMU IS OPERATIONAL ]-------------\n");
  }

  if (radio.begin())
  {
    radio.setDataRate( RF24_250KBPS );
    radio.setPALevel(RF24_PA_MIN);
    radio.enableAckPayload();
    radio.setRetries(3, 5); // delay, count
    radio.printDetails();
    delay(5000);
    // radio.openWritingPipe(slaveAddress); -- moved to loop()
    Serial.println("-------------[ RF COMMUNICATION ESTABLISHED ]-------------\n");
  }
  else
  {
    Serial.println("-------------[ RF COMMUNICATION FAILED ]-------------\n");
  }

}

void loop()
{
  // Iterate through slave devices in RF mesh
  for (int i = 0; i < numSlaves; i++)
  {
    // Read IMU Data
    IMU.readSensor();
    aX = (IMU.getAccelX_mss());
    aY = (IMU.getAccelY_mss());
    aZ = (IMU.getAccelZ_mss());
    gX = (IMU.getGyroX_rads());
    gY = (IMU.getGyroY_rads());
    gZ = IMU.getGyroZ_rads();
    mX = (IMU.getMagX_uT());
    mY = (IMU.getMagY_uT());
    mZ = (IMU.getMagZ_uT());

    // Store IMU data
    sendData[0] = aX;
    sendData[1] = aY;
    sendData[2] = aZ;
    sendData[3] = gX;
    sendData[4] = gY;
    sendData[5] = gZ;
    sendData[6] = mX;
    sendData[7] = mY;
    sendData[8] = mZ;
    
    // open the writing pipe with the address of a slave
    radio.stopListening();


    // Only one pipe can be open at once, but you can change the pipe you'll listen to.
    // Do not call this while actively listening.
    // Remember to stopListening() first.
    radio.openWritingPipe(slaveAddress[i]);


    radio_result = radio.write( &sendData, sizeof(sendData) );

    if (radio_result)
    {
      printData();
      if ( radio.isAckPayloadAvailable() )
      {
        //printData();
        radio.read(&ackData, sizeof(ackData));
      }
      else
      {
        //Serial.println("Acknowledge but no data ");
      }
    }
    else
    {
      Serial.println("Transmission failed");
    }
    //acknowledge_Data();
    //Serial.println();

  } // End of 'for(int i = 0; i < number_Slaves; i++)'
  delay(100);
}
