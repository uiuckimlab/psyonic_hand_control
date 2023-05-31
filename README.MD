# Psyonic Ability Hand ROS interface

This repository contains code to interface with the Psyonic Ability Hand through ROS and the Teensy 4.0

# Hardware Architecture 
The hardware of the system consist of a 24 volt power source. Using a [DCDC converter](https://www.amazon.com/Anmbest-Converter-Adjustable-Regulator-Protection/dp/B0B1CR2BNV/ref=sr_1_1?crid=3AJOJ104AQNX2&keywords=dcdc%2Bconverter%2B24v%2Bto%2B7.5v%2B15A&qid=1671468238&sprefix=dcdc%2Bconverter%2B24v%2Bto%2B7.5v%2B15a%2Caps%2C98&sr=8-1&th=1), we supply 7.4 volts to the Psyonic Ability Hand. The [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) is connected and powered by the operating pc by usb, and communicates with the Psyonic Ability Hand through UART. 

The Teensy runs at a loop rate of 500 hz. 


# Software Diagram
For the software, we are using ROS Noetic and rosserial to communicate the messages from the Teensy board over ROS. 

![TeensyAndPsyonic](https://github.com/uiuckimlab/psyonic_hand_control/assets/19510003/79ccbed3-51ca-4e0c-9635-e6355fff4ba9)

# Hand Moving Video

In this video we show the Psyonic Hand moving through ROS. The fingers are moving in a sinusoidal trajectory with a loop rate of 100 hz, and a range of 0 to 114 degrees.

https://github.com/uiuckimlab/psyonic_hand_control/assets/19510003/021323b0-3def-4230-9974-72d94442d86a

