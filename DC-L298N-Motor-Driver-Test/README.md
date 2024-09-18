# Driver-Test-MotorDC

- Description

This project tests a DC motor driver using a potentiometer to control the motor's speed. By reading the analog value from the potentiometer, the DC motor’s speed and direction can be dynamically controlled.

## Components Used

- Arduino Uno
- DC Motor
- Potentiometer
- Motor driver (e.g., L298N)
- Jumper wires

## Pin Connections

- POT_PIN (A0): Analog pin for reading the potentiometer value.
- enA (9): PWM pin for controlling motor speed.
- in1 (7): Pin for controlling motor direction (clockwise rotation).
- in2 (8): Pin for controlling motor direction (clockwise rotation).

## How It Works

- The potentiometer is used to control the speed of the DC motor.
- The analog value from the potentiometer is read via pin A0, then mapped to a range of 0-255 and converted into a PWM signal sent to pin enA.
- The motor rotates in a clockwise direction, and the speed is controlled by the potentiometer value.
- The motor speed is updated every 3 seconds.

## Usage Instructions

- Connect all components as per the pin diagram mentioned above.
- Upload the code to your Arduino using the Arduino IDE.
- Rotate the potentiometer to change the DC motor speed. The potentiometer value will be read and displayed on the Serial Monitor.

## Output

The motor will rotate clockwise, and the speed can be controlled via the potentiometer. The speed value will be displayed in the Serial Monitor every 3 seconds.
