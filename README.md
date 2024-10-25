# Introduction
This project contains schematics, script to create a bluetooth 3D mouse (Air Mouse) using Seeeduino ESP32C6 wifi/bluetooth and MPU9250/6500

# Parts
- Seeduino ESP32C6
- MPU 9250/6500 6 axis
- PCB 2cm x8cm
- LiPO 3.7 V battery
- Mouse switches 

# Wiring

| **ESP32C6**         | **MPU9250/6500**    | **LEFT BUTTON**    |**RIGHT BUTTON**    |
|-----------------|-----------------|-----------------|-----------------|
| GND | GND | NO | NO |
| 3.3V | VCC  | | |
| GPIO23  | SCL | | |
| GPIO22 | SDA | | |
| GPIO16 |  | C| |
| GPIO17 |  | |C |

# Battery
Beneath the body of ESP32C6 you can see 2 contact points for battery, they're Bat+ and Bat-, just connect Bat- to the battery's ground (black wire)
For positive of battery, you can create a close/open circuit by a 2 state switch to turn of the air mouse when not in use.


# Disclaimer
This project is under development and lack of features, please consider using it as a starting point for your own project

Please include @bluecyLabs into the shared content if you are using this script.