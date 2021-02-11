# rotary-inverted-pendulum

[![Build Status](https://circleci.com/gh/mc18g13/rotary-inverted-pendulum.svg?style=svg)](https://circleci.com/gh/mc18g13/rotary-inverted-pendulum)

A learning aid for control system design. A rotary inverted pendulum designed with 3d printable frame.

![](https://github.com/mc18g13/rotary-inverted-pendulum/blob/main/designs/rotary-inverted-pendulum.png?raw=true)

## Hardware

* Raspberry Pico
* CQRobot 11.7:1 Metal Gearmotor
* L298N motor driver
* AS5600 magnetic encoder
* 5mm diameter steel rod 20cm
* 2 * 625-2RS 5x16x5mm Bearing
* 4 * M3 bolts (box lid)
* 2 * M3 bolts (encoder mount)
* 2 * M3 bolts (pendulum arm mount)
* 2 * M2.5 bolts (motor mount)
* 2 * M4 bolts and nuts (box body mount)
* Male to Female and Female to Female Jumper wires

# Pico Firmware

This uses docker to setup an image with the required dependencies for building the firmware. Its driven by targets in a makefile as described below

## First Time Setup

This will setup the docker container for building the firmware

```
make
```

## Build
```
make build
```

## Upload
```
make upload
```

## Connect over usb
```
make usb
```
