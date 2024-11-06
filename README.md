# Invertor-stm32f476rct6
This repository provides a solution for generating and controlling two PWM signals using GPIO pins on a selected microcontroller board. The signals are intended for controlling an inverter (H-bridge), with adjustable frequency and duty cycle parameters.

Key Features:
- GPIO Pin Configuration: The system supports the configuration of two GPIO pins.
- PWM Signal Generation: The two GPIO pins will output PWM signals with configurable:
- Frequency: Adjustable between 10 Hz and 10 kHz.
- Duty Cycle: Adjustable from 0% to 100%.
- UART Interface for Parameter Adjustment: The system allows for parameter modification via UART communication. Commands sent through the UART interface can be used to adjust the frequency and duty cycle of the PWM signals.
### For example:
F:1000,D:40;

F:1000 – Set frequency to 1000 Hz.

D:40 – Set duty cycle to 40%.
