# F.E.I.N (Fully Embedded Inertial Navigator)


## Overview

The Fully Embedded Inertial Navigator, or F.E.I.N for short, is an all terrain personal navigation system that is designed with you in mind, to keep you safe and on track.

## Features

- Inertial Navigation via IMU
- Fall Detection
- Orientation Detection

## Setup & Assembly

1. Connect the MPU6050, SDA to 04 and SDL to 05
2. Connect an LED with a resistor to port 02
3. Connect a push button to port 01
4. Change the IP in `IMUESP32/IMUESP32.ino` to the IPv4 Address that is local to your network (do ipconfig in the terminal if on Windows and if you're on Linux then you already know how to find your IPv4 Address)
5. Change the ssid and password to the ssid and password associated with your network in `IMUESP32/IMUESP32.ino`
6. Flash `IMUESP32/IMUESP32.ino` to the ESP32 via Arduino IDE or PlatformIO
7. Make sure your computer running `app/main.py` or `app/net.py` is connected to the same network as the ESP32
8. Run `app/main.py` to see the plots of the points as you move the IMU around
9. Run `app/net.py` and violently move the breadboard/IMU connection around to see the distress signal being released. (Or just press the "emergency" button that was set up earlier)
10. Try running the ESP wirelessly with a power bank and see how you can walk around and still can have full access to features.
11. Enjoy :)

## Inspiration

The idea for an inertial navigation system (I.N.S.) came from our love for outdoor activities. A crisis can arise in the wilderness at any time, and being prepared is key. Getting lost, being stuck due to the elements, suffering an accident, etc are all things people experience. So we used our engineering skills to create a product that helps hikers and explorers maintain peace of mind during their outdoor adventures.


## What it does

F.E.I.N. provides a navigational interface that tracks your movement without relying on a GPS signal, and helps you get rescued if you find yourself in a bind. It measures your movement using physical sensors and plots the path you're taking on a screen, allowing you to backtrack if you find yourself off trail. It also features a beacon system that allows the user to send a distress signal in the event of an accident, allowing first responders to locate the user faster and more efficiently. FEIN also includes features like orientation identification and fall detection.


## How we Built it

We started out with an MPU-6050, which is a 6 axis IMU and took a dead reckoning approach to approximate change in position and orientation using the ESP32-c3 microcontroller. We applied the Kalman filter to improve our data accuracy, and used UDP (User Datagram Protocol) to transmit this processed data over wifi to our display which uses a python based GUI to plot this data on a map. We developed a matlab simulation that shows orientation and translation in a 3d space to fine tune our sensors and detect error sources. Lastly, to implement the beacon distress signal, we used an LED to physically indicate the beacon activation, and send the distress signal over UDP to a computer, due to a lack of a radio module which would be used in a real setting to transmit over long distances to first responders.


## Challenges we ran into

We ran into a significant number of challenges, especially relating to sensor error and inherent problems with dead reckoning to approximate position that arise due to the double integration of source data. We attempted to mitigate these issues by using the Kalman filter to reduce noise, while also calculating offset vectors to fine tune orientation and position. However, the sensors we used have an in built offset on top of drift based error, which made it difficult to fine tune our offset values. These problems on top of the issues that exist with long distance dead reckoning, made it quite challenging to acquire useful data. We have come up with some ideas to mitigate this but did not have enough time to try them out.

Another major issue we ran into was with our GPS module, which we wanted to use for initial position estimation, for a starting reference point. However, due to range limitations, we were unable to consistently replicate this use. We were able to get clean data after a significant amount of troubleshooting, but by this point, we didn't have enough time to write code to calculate position and use that in a reasonable capacity in our implementation.


## Accomplishments that we're proud of


We are proud of the fact that we were able to fine tune our filters enough to get relatively good data, and do all of this data processing on board our microcontroller, programmed in C and C++. We were able to maximize the use of our limited resources to produce a working model, despite a large number of hardware related issues.

