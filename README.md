# AI Driven Self-Driving Car Platform

# Introduction

This repository contains the source code for all software components of a self-driving car platform. The primary focus areas are as follows:
1. Software to drive roubst hardware platform to study autonomous vehicles providing a unique driving experience (WYSIWYG RXTX)
2. Study of CNN based neural networks for a self-driving R/C car
3. Image augmentation to improve neural network performance in autonomous vehicles

# Folder structure
- **unoTest** - Testing OLED, ultrasonic distance, pressure switch & joystick peripherals on an Arduino Uno prior to constructing the RX and TX modules.
- **rx** - The source code of the reciever module based on a Adafruit M0 Feather RFM69. It uses I2C and UARTG to communicate with the SBC (Raspberry Pi or other). Peripheals it uses are: 16x2 LCD on I2C, PCA9685 PWM driver on I2C, relay to control SBC power, 4 HR-SC04 ultrasonic sensors, gyroscope on I2C and magnetometer on I2C. It receives commands from the TX module.
- **tx** - The source coude of the transmitter module based on a Adafruit M0 Feather RFM69. It uses inputs from a joystick module and a force sensor. It uses a 128x64 I2C OLED LED module, 10 segment LED bar graph, buttons, 2 color LEDs, 1 RGB LED and a buzzer for output. It sends commands to the RX module.
- **pi/host** - On the Raspberry Pi, it is the communicator between the SBC and RX. It sets the state of the SBC on RX, (is envisioned to) host Python scripts (other processes) that run for AI training, control and other purposes. opens a named FIFO to pass data to such processes on one thread and recieves and transmits control commands and events to and from RX.
- **pi/progs/mqttClient** - A sample program written to test the I2C communication when the SBC (Raspberry Pi) acts as master. It receives commands via MQTT and controls the steering and throttle when the host has been started as master and the RX has stopped sending sensor data on I2C slave.
- **pi/progs/trainData** - This program runs when the SBC is in passive mode and is used to capture image frames, resize them and store them as images with a timestamp. It parallelly reads the sensor and control input from the FIFO produced by pi/host and stores it in a file in the same folder as the images.
- **training** - The main.py in this folder is used to train a CNN based neural network model with the training images recorded with pi/progs/trainData and uses the control inputs as labels. It then displays the training loss graph as epochs progress.
- **pi/progs/pilot1** - This program takes a trained model as input and runs in master mode sending commands to the PWM module. It is the self driver. It can either capture images live from the Raspberry Pi camera or can be provided a folder of images for simulated driving. It shows the current video frame and useful data.
- **hw** - This contains the hardware schematic (not completed) and the CAD files for mounting the hardware on the Tamiya TT02 chassis.

# WYSIWYG RXTX

A part of this project (WYSIWYG RXTX) won the [Melbourne Raspberry Pi Makers Group's 2004 Makers Challenge](https://melbourne-rpi.com.au/competition/)
