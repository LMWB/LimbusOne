# LimbusOne
Custom Microcontroller Board based on STM32F042-Series

![front](/Images/LimbusOne1.png)
![back](/Images/LimbusOne2.png)
![top](/Images/LimbusOne3.png)
[schematics](/Hardware/LimbusOne/PDF/LimbusOne.pdf)

 Features
- STM32F042K6T6 Microcontroller
- it got a crystal oscillator but I did not use it
    - accuracy is good since USB Data is working
- Power indicator LED
- User LED
- User Push Button
- Industrial ready CAN-Bus interface
    - works with standard (11-Bit) and extended (29-Bit) CAN identifier
- Industrial ready Modbus interface
- USB mini as power supply AND DATA
    - unfortunately USB and CAn share same Pin's so the user has to decide what to use
- 2.54mm Pin Headers for debugging and tracing
- debug / programming interface according to STLink that comes with most of STM32 Nucleo Boards

# Todo
- adjust Pin Header spacing to a more usable spacing, right know its pretty random