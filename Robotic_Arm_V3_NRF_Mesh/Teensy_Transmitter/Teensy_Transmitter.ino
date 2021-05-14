/*
   OLED Display using I2C communication BUS

   If using Teensy 4
   SCL = Pin 19
   SDA = Pin 18

*/
#include "MPU9250.h"
#include <Servo.h>
#include "U8g2lib.h"
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

#define CE_PIN  7
#define CSN_PIN 8
#define NANO 0
#define WEDMOS 1

U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
MPU9250 IMU(SPI, 10);
Servo x_servo, y_servo;

//---------------------- Variables ----------------------//
const int buttonPin = 2;         // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
int status, gyro_Calibration_Status,
    accel_Calibration_Status, magn_Calibration_Status,
    map_X, map_Y = 0;
static const float R = 50;                            // Noise covariance (Should be 10) | Higher R --> Filters more, but slower
static const float H = 1.00;                          // Measurement map scalar
static const float Q = 10;                            // Initial estimated covariance
float aX, aY, aZ, gX, gY, gZ, K, P, U_hat_gX, U_hat_gY, U_hat_gZ, U_hat_aX, U_hat_aY, U_hat_aZ, U_hat_mX, U_hat_mY, U_hat_mZ,
      filtered_gX, filtered_gY, filtered_gZ, filtered_aX, filtered_aY, filtered_aZ, gX_bias, gY_bias, gZ_bias,
      aX_bias, aY_bias, aZ_bias, roll, pitch, heading, set_gX_bias, set_gY_bias, set_gZ_bias,
      hxs, hys, hzs, hxb, hyb, hzb, gxb, gyb, gzb, axb, ayb, azb, axs , ays, azs, mag_x, mag_y, yaw,
      mX, mY, mZ, f_ax, filtered_roll, filtered_pitch, filtered_yaw, filtered_mX, filtered_mY, filtered_mZ,
      filtered_mag_x, filtered_mag_y, U_hat_roll, U_hat_pitch, U_hat_yaw = 0;
float sendData[5];
char display_pitch[20], display_roll[20], display_yaw[20], display_map_X[20], display_map_Y[20];

// NRF Stuff
bool radio_result;
const byte numSlaves = 2;
const byte slaveAddress[numSlaves][5] = {
  // each slave needs a different address
  {'R', 'x', 'A', 'A', 'A'},
  {'R', 'x', 'A', 'A', 'B'}
};
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
float dataToSend[100];
int ackData[2] = { 0, 0}; // to hold the two values coming from the slave
bool newData = false;
//---------------------- Functions ----------------------//
float K_filter_gyro(float &raw_gX, float &raw_gY, float &raw_gZ, float &filtered_gX, float &filtered_gY, float &filtered_gZ)
{
  K = P * H / (H * P * H + R);                        // Current Kalman gain = 0.36
  U_hat_gX = U_hat_gX + K * (raw_gX - H * U_hat_gX);  // Update Estimated values
  U_hat_gY = U_hat_gY + K * (raw_gY - H * U_hat_gY);
  U_hat_gZ = U_hat_gY + K * (raw_gY - H * U_hat_gY);
  P = (1 - K * H) * P + Q;                            // Update Error in Estimate
  filtered_aX = U_hat_aX;
  filtered_aY = U_hat_aY;
  filtered_aZ = U_hat_aZ;
}

float K_filter_accel(float &raw_aX, float &raw_aY, float &raw_aZ, float &filtered_aX, float &filtered_aY, float &filtered_aZ)
{
  K = P * H / (H * P * H + R);                        // Update Kalman Gain | KG == Large --> error in estimate == LARGE error in measurement == Small | KG == Small --> Error im measurement == Large Error in estimate == Small
  U_hat_aX = U_hat_aX + K * (raw_aX - H * U_hat_aX);  // Update Estimated values
  U_hat_aY = U_hat_aY + K * (raw_aY - H * U_hat_aY);
  U_hat_aZ = U_hat_aZ + K * (raw_aZ - H * U_hat_aZ);
  P = (1 - K * H) * P + Q;                            // Update Error in Estimate

  filtered_aX = U_hat_aX;
  filtered_aY = U_hat_aY;
  filtered_aZ = U_hat_aZ;

}

void read_Sensor()
{
  // Read the sensor
  IMU.readSensor();
  aX = IMU.getAccelX_mss();
  aY = IMU.getAccelY_mss();
  aZ = IMU.getAccelZ_mss();
  gX = IMU.getGyroX_rads();
  gY = IMU.getGyroY_rads();
  gZ = IMU.getGyroZ_rads();
  mX = IMU.getMagX_uT();
  mY = IMU.getMagY_uT();
  mZ = IMU.getMagZ_uT();
}


void K_filter(float &raw_aX, float &raw_aY, float &raw_aZ, float &raw_mX, float &raw_mY, float &raw_mZ, float &raw_gX,
              float &raw_gY, float &raw_gZ, float &filtered_aX, float &filtered_aY, float &filtered_aZ,
              float &filtered_gX, float &filtered_gY, float &filtered_gZ, float &filtered_mX, float &filtered_mY, float &filtered_mZ)
{
  K = P * H / (H * P * H + R);                        // Update Kalman Gain | KG == Large --> error in estimate == LARGE error in measurement == Small | KG == Small --> Error im measurement == Large Error in estimate == Small

  //Accelermotor
  U_hat_aX = U_hat_aX + K * (raw_aX - H * U_hat_aX);  // Update Estimated values
  U_hat_aY = U_hat_aY + K * (raw_aY - H * U_hat_aY);
  U_hat_aZ = U_hat_aZ + K * (raw_aZ - H * U_hat_aZ);

  //Gyroscopic
  U_hat_gX = U_hat_gX + K * (raw_gX - H * U_hat_gX);  // Update Estimated values
  U_hat_gY = U_hat_gY + K * (raw_gY - H * U_hat_gY);
  U_hat_gZ = U_hat_gY + K * (raw_gY - H * U_hat_gY);

  //Mag
  U_hat_mX = U_hat_mX + K * (raw_mX - H * U_hat_mX);  // Update Estimated values
  U_hat_mY = U_hat_mY + K * (raw_mY - H * U_hat_mY);
  U_hat_mZ = U_hat_mY + K * (raw_mY - H * U_hat_mY);
  P = (1 - K * H) * P + Q;

  filtered_aX = U_hat_aX;
  filtered_aY = U_hat_aY;
  filtered_aZ = U_hat_aZ;

  filtered_gX = U_hat_gX;
  filtered_gY = U_hat_gY;
  filtered_gZ = U_hat_gZ;

  filtered_mX = U_hat_mX;
  filtered_mY = U_hat_mY;
  filtered_mZ = U_hat_mZ;
}


void K_filter_pitch_roll(float &raw_roll, float &raw_pitch, float &raw_yaw)
{
  K = P * H / (H * P * H + R);                        // Update Kalman Gain | KG == Large --> error in estimate == LARGE error in measurement == Small | KG == Small --> Error im measurement == Large Error in estimate == Small

  U_hat_roll = U_hat_roll + K * (raw_roll - H * U_hat_roll);  // Update Estimated values
  U_hat_pitch = U_hat_pitch + K * (raw_pitch - H * U_hat_pitch);
  U_hat_yaw = U_hat_yaw + K * (raw_yaw - H * U_hat_yaw);

  P = (1 - K * H) * P + Q;

  filtered_roll = U_hat_roll;
  filtered_pitch = U_hat_pitch;
  filtered_yaw = U_hat_yaw;
}

void plot_serial_data()
{
  Serial.print(roll);  Serial.print(',');
  Serial.print(pitch); Serial.print(',');
  Serial.print(yaw); Serial.print(',');
  Serial.print(filtered_roll); Serial.print(',');
  Serial.print(filtered_pitch); Serial.print(',');
  Serial.println(filtered_yaw);
}
void display_serial_data()
{
  Serial.print("Roll : "); Serial.print(roll);
  Serial.print("\tPitch : "); Serial.print(pitch);
  Serial.print("\tYaw : "); Serial.print(yaw);
  Serial.print("\t Filtered Roll : "); Serial.print(filtered_roll);
  Serial.print("\tFiltered Pitch : "); Serial.print(filtered_pitch);
  Serial.print("\tFiltered Yaw  : "); Serial.println(filtered_yaw);
}
void setup()
{
  // Serial communication
  Serial.begin(115200);

  // Button stuff
  pinMode(buttonPin, INPUT);

  // OLED stuff
  OLED.setI2CAddress(0x3D * 2);
  OLED.begin();
  OLED.setFont(u8g2_font_7x13B_mr);

  // RF stuff
  radio.begin();

  //speed  RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setDataRate( RF24_250KBPS );

  //Ack payloads are a handy way to return data back to senders without manually changing the radio modes on both units.
  radio.enableAckPayload();

  /*
     Set the number and delay of retries upon failed submit.
     Syntax : setRetries(delay,count);
     delay How long to wait between each retry, in multiples of 250us, max is 15. 0 means 250us, 15 means 4000us.
     count How many retries before giving up, max 15
  */
  radio.setRetries(3, 5); // delay, count


  Serial.println("-------------[ SERIAL COMMUNICATION ESTABLISHED ]-------------\n");
  Serial.println("-------------[ CHECKING IMU ]-------------\n");

  // Start communication with IMU and estimates/removes gyro bias from sensor data | This has to be first for some reason
  status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  else
  {
    Serial.println("-------------[ IMU IS OPERATIONAL ]-------------\n");
    OLED.drawStr(40, 10, "Check");
    OLED.drawStr(40, 30, "Serial");
    OLED.drawStr(40, 50, "Monitor");
    OLED.sendBuffer();  // transfer internal memory to the display
    OLED.clearBuffer(); // clear the internal memory
  }

  Serial.println("Enter '1' to calibrate | Enter '2' to skip calibration");
  while (true)
  {
    buttonState = digitalRead(buttonPin);

    if ((Serial.available() > 0) || (buttonState == HIGH))
    {
      char user_input = Serial.read();
      if (user_input == '1')
        //switch (user_input)
      {
        //case '1':
        {
          // =============================================== CALIBRATION STUFF STARTS HERE =============================================== //

          //---------------------------------------------------- [ CALIBRATE GYRO ] ----------------------------------------------------//
          /*
              -- GYRO --
              The gyro bias is automatically estimated during the begin() function, then removed from sensor measurements
              IMU.calibrateGyro(); --> Will re-estimate gyro bias and remove new bias from future measurements
              NEEDS TO BE STATIONARY
              Function should return POSITIVE value on succcess | NEGATIVE value on failure
          */

          Serial.println("===============================================================================================");
          Serial.println("\nCALIBRATE GYROSCOPE...");

          // Wait for user input
          while (true)
          {
            // Checks serial monitor for incoming data
            if (Serial.available() > 0)
            {
              // If user presses 'ENTER' break out of while loop
              if (Serial.read() == '\n')
              {
                Serial.println("\nPlace IMU sensor on solid surface and DON'T TOUCH IT");
                delay(2000);

                Serial.println("\nCALIBRATION STARTING IN...\n");
                break;
              }
            }
            //Serial.print(".");
            delay(2000);
          }
          // Countdown timer
          for (int i = 5; i > 0; i--)
          {
            Serial.print(i);
            Serial.println("...");
            delay(1000);
          }
          Serial.println("\nCALIBRATION IN PROGRESS");

          if (IMU.calibrateGyro() == true)
          {
            Serial.println("\nCALIBRATION COMPLETE\n");

            // "Grab" bias values
            gxb = IMU.getGyroBiasX_rads();
            gyb = IMU.getGyroBiasY_rads();
            gzb = IMU.getGyroBiasZ_rads();

            // Set bias values
            IMU.setGyroBiasX_rads(gxb);
            IMU.setGyroBiasY_rads(gyb);
            IMU.setGyroBiasZ_rads(gzb);

            // Print out bias values
            Serial.println("---- GYRO BIAS VALUES SET TO ----\n");
            Serial.print("GXB : "); Serial.print(gxb); Serial.print(',');
            Serial.print(" GYB : "); Serial.print(gyb); Serial.print(',');
            Serial.print(" GZB : "); Serial.println(gzb);
          }
          //---------------------------------------------------- [ CALIBRATE ACCELEROMOTOR ] ----------------------------------------------------//

          /*
              -- ACCEL --
              int calibrateAccel() --> estimate bias/scale factor needed to calibrate accelerometer

              Works on one axis at a time, needs to run on all 6 sensor orientation
              After it collects enough sensor data, it'll estimate the bias/scale factor for all three accelerometer channels and apply the corrections to measured data
              Accel calibration needs to be performed ONCE on IMU
              Both get/set functions can be used to retrieve estimated bias/scale factors and use them during future power cycles or operations with IMU
          */
          Serial.println("===============================================================================================\n");
          Serial.println("\n-------------[ CALIBRATING ACCELEROMOTOR ]-------------\n");
          delay(1200);
          Serial.println("Orientate IMU to desiered INITAL position to calibrate all SIX axes\n");
          delay(1200);
          Serial.println("To initiate ACCELEROMOTOR calibration press [ ENTER ]...");
          // Wait for user input
          while (true)
          {
            // Checks serial monitor for incoming data
            if (Serial.available() > 0)
            {
              // If user presses 'ENTER' break out of while loop
              if (Serial.read() == '\n')
              {
                Serial.println("\nCALIBRATION IN PROGRESS...");
                break;
              }
            }
            Serial.print(".");
            delay(1000);

          }
          // This is where the actual calibration takes place. Orientate the IMU sensor in SIX different directions AND MAKE SURE IT'S STABLE
          for (int n = 1; n <= 6; n++)
          {
            Serial.print("\nCurrent calibration iteration(s) : ");
            Serial.print(n); Serial.println(" \\ 6\n");

            // Countdown timer
            for (int i = 5; i > 0; i--)
            {
              Serial.print(i);
              Serial.println("...");
              delay(1000);
            }
            Serial.println("\n-------------[ CALIBRATION IN PROGRESS ]-------------");

            IMU.calibrateAccel();

            Serial.println("\n-------------[ Change IMU posistion ]-------------");
          }
          Serial.println("\n-------------[ Calibrating accelerometer COMPLETE ]-------------\n");


          // Get Scale factor values
          // Scale factors are the ratio between the measured output and the change in the 'sense' input
          // More info about scale factors can be found here : https://www.edn.com/evaluating-inertial-measurement-units/
          axs = IMU.getAccelScaleFactorX();
          ays = IMU.getAccelScaleFactorY();
          azs = IMU.getAccelScaleFactorZ();

          // Biased Factors
          axb = IMU.getAccelBiasX_mss();
          ayb = IMU.getAccelBiasY_mss();
          azb = IMU.getAccelBiasZ_mss();

          // Display scale/bias values
          Serial.println("\n-------------[ SCALE FACTORS ]-------------\n");

          Serial.print("X_Scale Factor : "); Serial.print(axs);
          Serial.print(" Y_Scale Factor : "); Serial.print(ays);
          Serial.print(" Z_Scale Factor : "); Serial.println(azs);

          Serial.println("\n-------------[ BIAS VALUES]-------------\n");

          Serial.print("X_Biased : "); Serial.print(axb);
          Serial.print(" Y_Biased : "); Serial.print(ayb);
          Serial.print(" Z_Biased : "); Serial.println(azb);

          // Store scale/bias values into an array
          const float accelBias[3] = {axb, ayb, azb};
          const float accelFactor[3] = {axs, ays, azs};

          // Set the scale/bias values accordingly
          Serial.println("\n-------------[ IMU VALUES SET TO]-------------\n");
          IMU.setAccelCalX(accelBias[0], accelFactor[0]);
          IMU.setAccelCalY(accelBias[1], accelFactor[1]);
          IMU.setAccelCalZ(accelBias[2], accelFactor[2]);

          // Display
          Serial.print("Ax_Bias : "); Serial.print(accelBias[0]);  Serial.print(" | Ax_Accel : "); Serial.println(accelFactor[0]);
          Serial.print("Ay_Bias : "); Serial.print(accelBias[1]);  Serial.print(" | Ay_Accel : "); Serial.println(accelFactor[1]);
          Serial.print("Az_Bias : "); Serial.print(accelBias[2]);  Serial.print(" | Az_Accel : "); Serial.println(accelFactor[2]);
          delay(3000);

          Serial.println("\n===============================================================================================\n");
          delay(2000);

          //---------------------------------------------------- [ CALIBRATE MAGNETOMETER ] ----------------------------------------------------//

          Serial.println("\nPress [ ENTER ] to calibrate megnetometer...");

          // Calibration won't start until user inputs 'ENTER' in the serial monitor
          while (true)
          {
            // Checks serial monitor for incoming data
            if (Serial.available() > 0)
            {
              // If user presses 'ENTER' break out of while loop
              if (Serial.read() == '\n')
              {
                Serial.println("\n\n-------------[ CALIBRATING MAGNETOMETER WILL TAKE ~60-80 SECONDS TO COMPLETE...]-------------");
                delay(1500);
                Serial.println("\n-------------[ SLOWLY AND CONTINUOUSLY MAKE A FIGURE 8 MOTION DURING CALIBRATION ]-------------");
                delay(2000);

                Serial.println("\nCALIBRATION STARTING IN...\n");
                break;
              }
            }
            Serial.print(".");
            delay(1000);
          }

          // Countdown timer
          for (int i = 5; i > 0; i--)
          {
            Serial.print(i);
            Serial.println("...");
            delay(1000);
          }

          if (IMU.calibrateMag() == true)
          {
            // Gather magnetometer bias/scale factors
            hxb = IMU.getMagBiasX_uT();
            hyb = IMU.getMagBiasY_uT();
            hzb = IMU.getMagBiasZ_uT();
            hxs = IMU.getMagScaleFactorX();
            hys = IMU.getMagScaleFactorY();
            hzs = IMU.getMagScaleFactorZ();

            // Display bias/scale factors
            Serial.println("\nCALIBRATION COMPLETE\n");
            Serial.println("---- MAG BIAS VALUES ----\n");
            Serial.print("X_Bias : "); Serial.print(hxb); Serial.print(',');
            Serial.print(" Y_Bias : "); Serial.print(hyb); Serial.print(',');
            Serial.print(" Z_Bias : "); Serial.println(hzb);

            Serial.println("\n---- MAG SCALE VALUES ----\n");
            Serial.print("X_Scale : "); Serial.print(hxs); Serial.print(',');
            Serial.print("Y_Scale : "); Serial.print(hys); Serial.print(',');
            Serial.print("Z_Scale : "); Serial.println(hzs);

            // Set biased/scale factors
            IMU.setMagCalX(hxb, hxs);
            IMU.setMagCalY(hyb, hys);
            IMU.setMagCalZ(hzb, hzs);
            Serial.println("\n---- MAGNOMETER BIAS/SCALE FACTOR SET TO ----\n");

            Serial.print("Mag X Bias : "); Serial.print(hxb); Serial.print("\tMag X Scale Factor : "); Serial.println(hxs);
            Serial.print("Mag Y Bias : "); Serial.print(hyb); Serial.print("\tMag Y Scale Factor : "); Serial.println(hys);
            Serial.print("Mag Z Bias : "); Serial.print(hzb); Serial.print("\tMag Z Scale Factor : "); Serial.println(hzs);

          }
          // =============================================== CALIBRATION STUFF ENDS HERE =============================================== //
          Serial.println("\nPress [ ENTER ] to display IMU data...");

          // Calibration won't start until user inputs 'ENTER' in the serial monitor
          while (true)
          {
            // Checks serial monitor for incoming data
            if (Serial.available() > 0)
            {
              // If user presses 'ENTER' break out of while loop
              if (Serial.read() == '\n')
              {
                break;
              }
            }
            Serial.print(".");
            delay(1000);
          }
          break;
        }// End of first case statement
        if ((user_input == '2') || (buttonState == HIGH))
          //case '2':
        {
          // Gyro biased values
          IMU.setGyroBiasX_rads(-0.03);
          IMU.setGyroBiasY_rads(0.03);
          IMU.setGyroBiasZ_rads(0.04);

          // Set Accel scale/bias values
          axs = 0.10;
          ays = 0.10;
          azs = 0.23;

          axb = 1.00;
          ayb = 1.00;
          azb = 0.98;

          const float accelBias[3] = {axb, ayb, azb};
          const float accelFactor[3] = {axs, ays, azs};
          IMU.setAccelCalX(accelBias[0], accelFactor[0]);
          IMU.setAccelCalY(accelBias[1], accelFactor[1]);
          IMU.setAccelCalZ(accelBias[2], accelFactor[2]);

          // Set Mag values
          hxb = 23.05;
          hyb = -44.13;
          hzb = 43.40;

          hxs = 1.52;
          hys = 0.77;
          hzs = 0.95;
          IMU.setMagCalX(hxb, hxs);
          IMU.setMagCalY(hyb, hys);
          IMU.setMagCalZ(hzb, hzs);

          break;
        }// End of case 2
      }// End of switch() statement
      
      // Breaks out into main loop
      break;
      
    }// End of 'if (Serial.available() > 0)'
  }// End of 'while(true)'

  //Serial.println("Roll,Pitch,Yaw,Filtered_Roll,Filtered_Pitch,Filtered_Yaw");

}// End of void setup()

void loop()
{
  //https://howtomechatronics.com/tutorials/arduino/how-to-track-orientation-with-arduino-and-adxl345-accelerometer/
  //https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf

  // open the writing pipe with the address of a slave
  radio.stopListening();

  // Only one pipe can be open at once, but you can change the pipe you'll listen to.
  // Do not call this while actively listening.
  // Remember to stopListening() first.
  radio.openWritingPipe(slaveAddress[NANO]);
  radio.openWritingPipe(slaveAddress[WEDMOS]);

  // Gather sensor data from IMU
  read_Sensor();

  // Apply Kalman Filter to IMU values
  K_filter(aX, aY, aZ, mX, mY, mZ, gX, gY, gZ, filtered_aX, filtered_aY,
           filtered_aZ, filtered_gX, filtered_gY, filtered_gZ, filtered_mX, filtered_mY, filtered_mZ);

  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  //https://howtomechatronics.com/tutorials/arduino/how-to-track-orientation-with-arduino-and-adxl345-accelerometer/
  //https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf

  roll = atan(aY / sqrt(pow(aX, 2) + pow(aZ, 2))) * 180 / PI;
  pitch = atan(-1 * aX / sqrt(pow(aY, 2) + pow(aZ, 2))) * 180 / PI;
  mag_x = mX * cos(pitch) + mY * sin(roll) * sin(pitch) + mZ * cos(roll) * sin(pitch);
  mag_y = mY * cos(roll) - mZ * sin(roll);
  yaw = 180 * atan2(-mY, mX) / M_PI;

  K_filter_pitch_roll(roll, pitch, yaw);
  //plot_serial_data();
  //display_serial_data();

  map_X = map(filtered_roll, -80, 80, 0, 180);
  map_Y = map(filtered_pitch, -77, 82, 0, 180);

  x_servo.write(abs(map_X));
  y_servo.write(abs(map_Y));

  //Roll = -80 80
  //Pitch = -77 82

  sendData[0] = pitch;
  sendData[1] = yaw;
  sendData[2] = roll;
  sendData[3] = map_X;
  sendData[4] = map_Y;

  // SEE HOW MUCH DATA CAN BE WRITTEN AT ONE TIME
  radio_result = radio.write( &dataToSend, sizeof(dataToSend) );

  // Write data
  if (radio_result)
  {
    Serial.print("Pitch : "); Serial.print(sendData[0]);
    Serial.print("\tRoll : "); Serial.print(sendData[1]);
    Serial.print("\tYaw: "); Serial.print(sendData[2]);
    Serial.print("\tMap_X : "); Serial.print(sendData[3]);
    Serial.print("\tMax_Y : "); Serial.println(sendData[4]);
    if ( radio.isAckPayloadAvailable() )
    {
      radio.read(&ackData, sizeof(ackData));
      newData = true;
    }
    else
    {
      Serial.println("Acknowledge but no data ");
    }
  }
  else
  {
    Serial.println("Trasnmission FAILED");
  }
  Serial.println();

  /*
    Serial.print("Pitch : "); Serial.print(sendData[0]);
    Serial.print("\tRoll : "); Serial.print(sendData[1]);
    Serial.print("\tYaw: "); Serial.print(sendData[2]);
    Serial.print("\tMap_X : "); Serial.print(sendData[3]);
    Serial.print("\tMax_Y : "); Serial.println(sendData[4]);
  */
  // OLED STUFF
  dtostrf(roll, 2, 2, display_roll);
  dtostrf(pitch, 2, 2, display_pitch);
  dtostrf(yaw, 2, 2, display_yaw);
  dtostrf(map_X, 2, 2, display_map_X);
  dtostrf(map_Y, 2, 2, display_map_Y);

  radio.write(sendData, sizeof(sendData)); // sizeof(data) == 20 bytes

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

  /*
    OLED.drawStr(5, 10, "Pitch : ");
    OLED.drawStr(80, 10, display_pitch);

    OLED.drawStr(5, 35, "Roll : ");
    OLED.drawStr(80, 35, display_roll);

    OLED.drawStr(5, 60, "Yaw : ");
    OLED.drawStr(80, 60, display_yaw);
    OLED.sendBuffer();  // transfer internal memory to the display
    OLED.clearBuffer(); // clear the internal memory
    Taking the gyroscopic, accelerator, magometer data from a calibrated MPU_9250 to calculate the yaw, pitch and roll which is then sent via a wireless communication protocol to control a 4-Degree-Of-Freedom robotic arm, or other peripherals such as rovers, drones, ect.

  */

  delay(50);
}
