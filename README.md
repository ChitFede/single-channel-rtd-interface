# Single-Channel RTD Interface

This project implements a custom single-channel RTD (Resistance Temperature Detector) interface module that supports both Pt100 and Pt1000 RTDs using a 4-wire configuration. It includes hardware design, embedded firmware (Arduino/C++), and optional RTD type auto-detection using the MAX31865 chip over SPI.

## Features

- Supports Pt100 and Pt1000 RTDs (α = 3850 ppm/°C)
- 4-wire RTD configuration for accurate measurements
- SPI-based digital temperature readings
- Auto RTD type detection logic (optional)
- Fault detection and temperature calculation (Callendar-Van Dusen)
- Robust and reusable Arduino-compatible code
- Hardware schematic designed using EasyEDA

## Folder Structure

