# USB RC Joystick Adapter
This project turns Arduino Leonardo (or Pro Micro) into a USB adapter for RC transmitter.
The adapter can be used to play [FPV Freerider](http://fpv-freerider.itch.io/fpv-freerider) 
or other flight simulators. 

The project is based on [rc-leonardo-joy](https://github.com/i--storm/rc-leonardo-joy)
and contains the following improvements:
- higher sticks resolution (0-1000 instead of 0-255)
- sharper reading of ppm intervals (using timer input capture interrupt instead of _micros()_ func)
- cleaner and more specialized code 

Improvments by D. Gayraud:
- Use MHeironimus Joystick Library (https://github.com/MHeironimus/ArduinoJoystickLibrary) 
- Add ZAxis and RzAxis


#Flashing:
  Open the sketch in Arduino IDE, choose Leonardo board, upload.

#Connections:
- RC PPM out <==> Digital Pin 4 of Arduino Leonardo
- RC GND  <==> Arduino GND

#Usage:
In Windows: open *Devices and Printers* find *Arduino Leonardo*. Then right click on it and choose *Game controller settings*. Calibrate joystick using your RC transmitter connected.

Definitely works with Futaba 7C transmitter and FPV FreeRider simulator :)
