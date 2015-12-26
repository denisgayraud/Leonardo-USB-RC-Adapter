# rc-leonardo-joy
This project turns Arduino Leonardo (or Pro Micro) into a USB adapter for RC transmitter.
The adapter can be used to play [FPV Freerider](http://fpv-freerider.itch.io/fpv-freerider) 
or other flight simulators. 

The project is based on [rc-leonardo-joy](https://github.com/i--storm/rc-leonardo-joy)
and contains the following improvements:
- higher sticks resolution (0 - 1000 instead of 0 - 255),
- sharper reading of ppm intervals (using timer input capture interrupt instead of _micros()_ func)
- cleaner and more specialized code 

#Flashing:
Put "hid.cpp" and "usbapi.h" from ArduinoLibs folder into Arduino installation folder: 
<Arduino_Installation_Folder>\hardware\arduino\cores\arduino\  
Choose Leonardo board and simply flash it with the sketch

#Connections:
- RC PPM out <--> Digital Pin 4 of Arduino Leonardo
- RC GND  <--> Arduino GND

#Usage:
In Windows, find Arduino Leonardo in Devices and Printers. Then right click on it and choose Game device Parameters. Calibrate joystick there using your RC transmitter connected

Tested on Futaba 7C transmitter and FPV FreeRider simulator.
