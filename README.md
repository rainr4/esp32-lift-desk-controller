# ESP32 Lift Desk Controller

A Wi-Fi enabled controller for motorized standing desks using ESP32, featuring dual motor control with PID-based balancing and web interface.

## Features

- **Dual Motor Control**: Independent control of left and right motors with synchronized movement
- **PID Balancing**: Automatic motor balancing using hall sensor feedback to prevent uneven movement
- **Wi-Fi Connectivity**: Web-based control interface accessible from any device on the network
- **Hall Sensor Feedback**: Precise position tracking and motor synchronization
- **OTA Updates**: Over-the-air firmware updates for easy maintenance
- **Configurable Parameters**: Adjustable PID values, speed control, and balance intervals

## Hardware Requirements

- ESP32 Development Board
- 2x DC Motors with Hall Sensors
- Motor Driver (compatible with PWM control)
- Hall Sensor Modules (2 per motor)

## Pin Configuration

### Motor Control Pins
- Left Motor: PWM pins 25 (RPWM), 27 (LPWM)
- Right Motor: PWM pins 4 (RPWM), 2 (LPWM)

### Hall Sensor Pins
- Left Motor: Pins 36, 39
- Right Motor: Pins 16, 17

## Setup Instructions

1. **Hardware Setup**: Connect motors and hall sensors according to the pin configuration
2. **Software Setup**: 
   - Install PlatformIO IDE
   - Create `src/config.h` with your Wi-Fi credentials:
     ```cpp
     #define WIFI_SSID "Your_WiFi_SSID"
     #define WIFI_PASSWORD "Your_WiFi_Password"
     #define OTA_PASSWORD "Your_OTA_Password"
     ```
3. **Upload**: Use PlatformIO to compile and upload the firmware
4. **Web Interface**: Connect to the ESP32's web server to control the desk

## Configuration

Key parameters that can be adjusted in the code:

- **PID Parameters**: Kp, Ki, Kd values for motor balancing
- **Speed Control**: currentSpeed, minPWM, maxPWM values
- **Balance Check Interval**: Frequency of PID adjustments
- **Debounce Time**: Hall sensor debounce timing

## Web Interface

The built-in web server provides:
- Motor control buttons (Up/Down/Stop)
- Individual motor control for calibration
- Real-time hall sensor readings
- PID parameter adjustment
- System status monitoring

## Safety Features

- Motor braking system for smooth stops
- Debounced hall sensor inputs to prevent false readings
- Configurable speed limits
- Emergency stop functionality

## Development Environment

- **Platform**: PlatformIO
- **Framework**: Arduino
- **Board**: ESP32 Development Module
- **Language**: C++

## License

This project is open source. Feel free to modify and distribute according to your needs.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.
