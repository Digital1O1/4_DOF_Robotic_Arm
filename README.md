# Intention Of 4_DOF_Robotic_Arm Project
1. Take gyroscopic/magnetometer/accelerometer data from an MPU_9250 to calculate the Yaw/Pitch/Roll
2. Using three nrf24l01 radio frequency development boards to transmit data wirelessly from one microcontroller (Teensy 4) to two other microcontrollers (Wedmos D1 | Arduino Nano)
3. Display on an SH1106 OLED display the calculated Yaw/Pitch/Roll values on the Transmitting Teensy while transferring the data to the other microcontrollers mentioned earlier
4. The microcontrollers recieving the transmitted data will display the incoming data on another SH1106 OLED display while the other will be used to control a 3-d printed, 4 Degree of Freedom robotic arm. 

## Bill of Materials (BOM)
1. MINIMUM two microcontrollers of your choice. An explanation to why three were used in this project will be explained later on.
2. 2 or 3 [NRF24l01](https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_7?dchild=1&keywords=nrf24l01&qid=1620930837&sr=8-7) radio frequency development boards
3. A BUNCH of male-female wires
4. 10uF or 100 uF electrolytic capacitors
5. 2 SG-90 servo motors
6. 2 SH1106 OLED displays
