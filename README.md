# rotary-nverted-pendulum

[![Build Status](https://circleci.com/gh/mc18g13/rotary-inverted-pendulum.svg?style=svg)](https://circleci.com/gh/mc18g13/rotary-inverted-pendulum)

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
