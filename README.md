# 433MHz Radio Signal Capture with Raspberry Pi Pico

This project captures and analyzes 433MHz radio frequency signals using a Raspberry Pi Pico microcontroller. The Pico uses its PIO (Programmable I/O) to precisely measure pulse widths from a 433MHz receiver module, and transmits the data to a web-based viewer via USB serial communication.

## Features

- Precise pulse width measurement using Pico's PIO
- Real-time visualization of captured RF signals
- Web-based interface using Web Serial API
- Multiple signal visualization with color coding

## Hardware Requirements

- Raspberry Pi Pico
- 433MHz receiver module (typically connected to GPIO 18)
- Computer with Chrome or Edge browser (version 89+ for Web Serial API support)

## Software Requirements

- Raspberry Pi Pico SDK
- CMake
- GCC ARM Embedded Toolchain
- Web browser supporting Web Serial API (Chrome 89+, Edge 89+)

## Setup Instructions

### Building the Firmware

1. Clone or download this repository
2. Set up the Raspberry Pi Pico SDK
3. Create a build directory: `mkdir build && cd build`
4. Configure the build: `cmake ..`
5. Compile the firmware: `make`
6. Copy the resulting `.uf2` file to your Pico in bootloader mode

### Using the Web Interface

1. Connect your Pico to your computer via USB
2. Open [capture_viewer.html](./capture_viewer.html) in a compatible browser
3. Click "Connect to device" and select your Pico's serial port
4. Press buttons on your 433MHz remote control
5. Observe the captured signals visualized as waveforms

## Code Structure

- [rec.c](./rec.c): Main Pico firmware implementing pulse capture using PIO
- [pico_sdk_import.cmake](./pico_sdk_import.cmake): CMake script to import Pico SDK
- [capture_viewer.html](./capture_viewer.html): Web-based viewer for captured signals
- [README.md](./README.md): This file

## How It Works

The Pico firmware uses the PIO peripheral to accurately measure the duration of high and low pulses from the 433MHz receiver. These measurements are stored in a buffer and transmitted to the host computer via USB serial. The web interface receives this data and visualizes it as waveform diagrams, allowing analysis of the RF protocol used by remote controls or other devices.

## License

This project is licensed under the BSD 3-Clause License - see the LICENSE file for details.