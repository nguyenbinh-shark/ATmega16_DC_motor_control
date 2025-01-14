# Motor Control with ATmega16

This project demonstrates how to control a DC motor using the ATmega16 microcontroller. The system includes features such as speed control, direction reversal, stopping, and resetting, with real-time speed feedback displayed on an LCD.

## Features
- **Microcontroller**: ATmega16
- **Motor Driver**: L298
- **User Controls**:
  - Increase speed
  - Decrease speed
  - Reverse direction
  - Stop
  - Reset
- **Feedback System**:
  - DC motor equipped with an encoder to measure speed.
  - Speed is displayed on a 16x2 LCD screen.

## Components
- ATmega16 microcontroller
- L298 motor driver IC
- DC motor with encoder
- 16x2 LCD module
- Push buttons (5 buttons for control: increase speed, decrease speed, reverse direction, stop, reset)
- Resistors, capacitors, and other passive components
- Power supply

## Circuit Design
The circuit is designed to interface the ATmega16 microcontroller with the L298 motor driver and the encoder-equipped DC motor. The speed of the motor is measured using the encoder and displayed on the 16x2 LCD. The five push buttons are used to control the motor's operation.

### Circuit Features:
1. **Motor Control**: The L298 motor driver is connected to the ATmega16 to control the motor's speed and direction.
2. **Encoder Feedback**: Encoder pulses are read by the microcontroller to calculate and monitor the motor speed.
3. **LCD Display**: Real-time speed data is displayed on a 16x2 LCD module.

## Functionality
1. **Increase Speed**: Press the "Increase Speed" button to raise the motor speed.
2. **Decrease Speed**: Press the "Decrease Speed" button to lower the motor speed.
3. **Reverse Direction**: Press the "Reverse Direction" button to change the motor's rotation direction.
4. **Stop**: Press the "Stop" button to halt the motor.
5. **Reset**: Press the "Reset" button to restore the system to its initial state.

## How to Build
1. Assemble the circuit based on the provided schematic.
2. Upload the firmware to the ATmega16 microcontroller.
3. Power the circuit and test each functionality using the push buttons.
4. Monitor the motor speed on the LCD screen.

## Files Included
- Circuit design files (schematics and PCB layout).
- Firmware source code for ATmega16.

## Tools and Software
- **Programming**: Atmel Studio for coding and uploading firmware to the ATmega16.
- **Circuit Design**: Altium Designer or equivalent software for PCB design.
- **Simulation**: Proteus for simulating the circuit functionality.

## Future Improvements
- Add support for additional motor types.
- Implement PID control for better speed regulation.
- Enhance the user interface with more detailed feedback.

## License
This project is open-source and available under the MIT License. Feel free to modify and distribute.
