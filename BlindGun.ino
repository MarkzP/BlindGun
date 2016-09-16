
/*************************************************************************************
 * 
 * BlindGun - a MAME compatible lightgun that uses no light to operate
 * By Marc Paquette  marc @^ dacsystemes.com
 *
 * Uses highly customized DCM code from Razor AHRS  https://github.com/ptrbrtz/razor-9dof-ahrs
 * Runs on a Teensy LC from PJRC http://www.pjrc.com/store/teensylc.html
 * Relies on Sparkfun LSM9DS1 library https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
 * Compiled with Arduino 1.6.9 & Teensyduino http://www.pjrc.com/teensy/teensyduino.html
 *
 * If you build one (or two), let me know!
 * 
 * USB Type should be "Keyboard + Mouse + Joystick"
 * 
 * To calibrate the screen area:
 *  -Point to the upper left corner
 *  -Hold the trigger until the led comes on
 *  -Move to the lower right corner
 *  -Release the trigger
 *  -Don't move from where you are! This is a blind gun; really, it has nu idea what (or where) you are shooting at
 *  
 * To switch between joystick & mouse mode:
 *  -Hold both the trigger & button 1 until led flashes quickly
 *  
 * If it starts to drift or becomed somewhat unuseable, just put the gun down for ~10 sec. This will recalibrate the sensors.
 *
 * This code is released under GPL v3.0: https://gnu.org/licenses/gpl-3.0.txt
 * 
 *************************************************************************************/

bool useMouse = false;

//0.0 = top/left, 1.0 = bottom/right
float pos_x = 0;
float pos_y = 0;

bool calibrating = false;

float yaw, pitch, roll; //absolute orientation angles from AHRS

float relativeYaw;
float relativePitch;

float yawZero = 1.0f; //Where the screen start (in rads)
float yawRange = 0.3f; //How wide the screen is (in rads)
float pitchZero = 0.0f;
float pitchRange = 0.25f;

// Mag calibration: DONT'T SKIP THIS!
//
// When MAG_CALIB is defined, the sketch will behave just like in the NXPMotionSense\CalibrateSensors example.
// Use Paul Stoffregen excelent MotionCal software https://github.com/PaulStoffregen/MotionCal to compute & save the mag correction values to EEPROM. 
// Don't forget to set the USB Type = Serial + Keyboard + Mouse + Joystick
//
//#define MAG_CALIB
//
// If you don't have a mag, don't worry, it will still work, but it will drift a tiny bit.


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  initButtons();
  initOrientation();
  initSensors();
  initOutputDevice();

  //Flash!
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (dataAvailable()) {

    #ifdef MAG_CALIB
      doMagCalibration();
    #else
    
      updateOrientation();
      checkButtons();      
      sendPosition();
    #endif      
  }
}

