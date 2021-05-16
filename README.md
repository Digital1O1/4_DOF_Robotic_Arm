![DSC_0498](https://user-images.githubusercontent.com/39348633/118382828-ba5cf500-b5be-11eb-963e-74b74a1ba056.jpg)

# Intention Of 4_DOF_Robotic_Arm Project
1. Take gyroscopic/magnetometer/accelerometer data from an MPU_9250 to calculate the Yaw/Pitch/Roll
2. Using three nrf24l01 radio frequency development boards to transmit data wirelessly from one microcontroller (Teensy 4) to two other microcontrollers (Wedmos D1 | Arduino Nano)
3. Display on an SH1106 OLED display the calculated Yaw/Pitch/Roll values on the Transmitting Teensy while transferring the data to the other microcontrollers mentioned earlier
4. The microcontrollers recieving the transmitted data will display the incoming data on another SH1106 OLED display while the other will be used to control a 3-d printed, 4 Degree of Freedom robotic arm. 

## Bill of Materials (BOM)
1. Two microcontrollers of your choice. A Teensy 4.0, Wedmos D1 and an Arduino Nano were used in this instance for reasons explained later in this repo.
2. 2 or 3 [NRF24l01](https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_7?dchild=1&keywords=nrf24l01&qid=1620930837&sr=8-7) radio frequency development boards
5. A BUNCH of male-female wires
6. 10uF or 100 uF electrolytic capacitors
7. 2 SG-90 servo motors
8. 2 SH1106 OLED displays
9. 1 Push button

## Overview of Project

![image](https://user-images.githubusercontent.com/39348633/118383319-53d9d600-b5c2-11eb-9b31-1e49dd980473.png)

