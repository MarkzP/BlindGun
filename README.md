# BlindGun
a MAME compatible lightgun that uses no light to operate
By Marc Paquette  marc @^ dacsystemes.com

Uses highly customized DCM code from Razor AHRS  https://github.com/ptrbrtz/razor-9dof-ahrs
Runs on a Teensy LC from PJRC http://www.pjrc.com/store/teensylc.html
Relies on Sparkfun LSM9DS1 library https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
Compiled with Arduino 1.6.9 & Teensyduino http://www.pjrc.com/teensy/teensyduino.html

If you build one (or two), let me know!
 
USB Type should be "Keyboard + Mouse + Joystick"

To calibrate the screen area:
-Point to the upper left corner
-Hold the trigger until the led comes on
-Move to the lower right corner
-Release the trigger
-Don't move from where you are! This is a blind gun; really, it has nu idea what (or where) you are shooting at

To switch between joystick & mouse mode:
-Hold both the trigger & button 1 until led flashes quickly

If it starts to drift or becomed somewhat unuseable, just put the gun down for ~10 sec. This will recalibrate the sensors.
