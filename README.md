# Intention Of 4_DOF_Robotic_Arm Project
1. Take gyroscopic/magnetometer/accelerometer data from an MPU_9250 to calculate the Yaw/Pitch/Roll
2. Use three NRF24l01 Radio frequency development boards to wirelessly transmit data from one microcontroller (Teensy 4) to two other microcontrollers (Wedmos D1 | Arduino Nano)
3. Display the calulated Yaw/Pitch/Roll values that are being transmitted/recieved on their perspective SH1106 OLED display (Recieving/Transmitting)
4. Use one microcontroller that's recieving the transmitted data to control a 3-d printed, 4 Degree of Freedom robotic arm. 

![DSC_0498](https://user-images.githubusercontent.com/39348633/118382828-ba5cf500-b5be-11eb-963e-74b74a1ba056.jpg)

## Bill of Materials (BOM)
1. (2) microcontrollers of your choice. (3) were used (Teensy 4/Arduino Nano/ Wedmos D1) in this instance for reasons explained later in this repo.
3. (2) or (3) NRF24l01 radio frequency development boards
5. A BUNCH of male-female wires
6. (3) 10uF or 100 uF electrolytic capacitors
7. (2) SG-90 servo motors
8. (2) SH1106 OLED displays
9. (1) Push button

# In-depth Explantion of Project
- The user is prompted through the Arduino serial monitor or terminal of their choice if they want to calibrate the MPU_9250. 
    - If the user enters '1' in the serial monitor, the calibration procedure will initiate. 
    - If the user enters '2', they can skip the calibration process completely.
- The MPU_9250 gyroscopic/magnetometer/accelerometer data is sent via __SPI__ to the Teensy 4 in order to calculate the pitch/yaw/roll 
- The calculated values are then displayed on an SH1106 OLED display connected via __I2C__ on the Teensy 4 
- The same calculated values are then transmitted through the nrf24l01 to an Arduino Nano and an Wedmos D1 microcontroller as shown in the figure below.

![image](https://user-images.githubusercontent.com/39348633/118416160-f0a87c00-b673-11eb-946e-def25c10f72f.png)

- The Arduino Nano recieves the transmitted data from the Teensy 4 in order to control the 4-DOF robotic as shown in the screenshot below

![image](https://user-images.githubusercontent.com/39348633/118418023-54cf3e00-b67c-11eb-9844-b7e993eb8286.png) 

- The Wedmos D1 displays the recieved data on it's own SH1106 OLED display to give reassurnace that the transmitted data is being recieved from the Teensy 4.

## Kalman Filter
What???s a Kalman Filter?

An algorithm that estimates an unknown variable based on the measurements given over time.

Why use a Kalman Filter?

- The Kalman filter was used to reduce the noise from the gyroscopic/accelerometer/magnetometer data to allow the yaw/pitch/roll angles to be calculated as accurately as possible.

- Kalman filters are also typically used with drones to determine their orientation during flight.

- KEEP IN MIND, the values used for the Kalman filter have NOT been fine tuned??? yet since the majority of the time spent on this project was focused on other aspects of this project; but the values will be fine tuned at a later date.

# What's With All The Folders In The Repo?

Here's a quick run-down 

- I2C_Scanner
  - Used for troubleshooting peripherals that use the I2C communication protocol/bus
  - Peripherals like OLED displays use I2C
  - The 'I2C_Scanner' can be used to determine the address of the OLED display, or any peripheral using I2C.

- NRF Folder
  - Contains all NRF24l01 code
  - Hello World 
    - One microcontroller transmits 'Hello World' while the other microcontroller recieves it and displays it on the serial monitor
  - Transmit_Array | Transmit_Struct
    - As the name implies, the transmitted data can be sent two ways. As an array, or as a struct. In reference to this project, the array method was used
  - ackPayload
    - Two way communication between two microcontrollers
    - To elaborate further :
      - One microcontroller transmits data
      - The other recieves it and sends an 'Acknowledgement' back to the transmitting microcontroller that data was recieved
  
  - Robotic_Arm_V3_NRF_Mesh
    -  Teensy_Transmit_IMU_Data
        - Transmit only the gryo/accel/magometer data to the recieving microcontrollers (Wedmos D1/ Nano)
    - Teensy_Transmitter
      - Gives user the option to calibrate the MPU_9250 or to skip the calibration process
      - Calculate yaw/pitch/roll
      - Display yaw/pitch/roll data on SH1106 OLED display
      - Transmit yaw/pitch/roll datat to Wedmos/Nano microcontrollers
    - Nano_Recieve_Robotic_Arm
      - Recieves transmitted data from Teensy
      - Microcontroller responsible for controlling Robotic Arm
    - Wedmos_Recieve_Robotic_Arm
      - Recieves transmitted data
      - Display transmitted data on SH1106 OLED display

# Why Use Two Microcontrollers to Recieve the Transmitted Data?

The Arduino Nano/Uno has only 32 KB of on-board Flash memory.

When the Servo,RF,and OLED libraries were included in the Arduino Nano/Uno sketch/code, the majority of the Flash memory (~97%) was used which lead to the microcontroller to become unstable and would periodically crash. 

When the same steps were repeated with the Wedmos D1, the RF and OLED libraries together would work fine. 

However when the Servo libraries were included in the sketch/code, the watchdog timer was triggered for some reason and the program wouldn't run.

So rather than troubleshooting why the watchdog timer was being triggered, I took the easy way out of the entire situation and just had the Wedmos D1 display the data being recieved, while the Nano was being used to control the robotic arm and would display the transmitted data via the serial monitor.

# FAQ

- Is the Wedmos D1 needed?
  - Honestly, not really. As mentioned earlier, the sole purpose of the Wedmos D1 was to display the transmitted data from the Teensy.
  - A person really only needs the Teensy (Or a similar development board that has a clock speed greater than 80 Mhz) and the Arduino Uno/Nano to run everything

- Will you re-visit and troubleshoot why the Watchdog timer is being triggered on the Wedmos D1 when the servo library is inlucded with the RF and OLED libraries?
  - It'll depend on the amount of free time I have on my hands since I'm not working on projects in my Github Repository full time

- How will I print out the STL files needed for the Robotic Arm?
  - Contact your local library. Most of them nowadays have a "Maker Space" and SHOULD have 3d-printers that the public can use.
