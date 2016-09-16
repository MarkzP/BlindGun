//
// Here I'm using Sparkfun SEN-13284 9DoF IMU Breakout - LSM9DS1 https://www.sparkfun.com/products/13284
//
// However the code could be quickly ported to any 9dof platform
// The extra methods are there so the sensor can be calibrated using the MotionCal software

#include <SparkFunLSM9DS1.h>
#include <Wire.h>
#include <SPI.h>
#include <util/crc16.h>
#include <EEPROM.h>

#define INT1_PIN        2    //Gyro Data Ready Interrupt
#define CS_AG_PIN       4
#define CS_M_PIN        3

#define TO_RAD(x) (x * PI / 180.0f)

#define CAL_EEADDR  20
#define CAL_SIZE    68

//These values are used to convert raw values to the ranges expected by the calibration application.
#define G_PER_COUNT            0.0001220703125f  // = 1/8192
#define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
#define UT_PER_COUNT           0.1f


byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount=0;
int loopcount = 0;

LSM9DS1 imu;

bool dataAvailable()
{
  #ifdef INT1_PIN
    return digitalRead(INT1_PIN) == LOW;
  #else
    return imu.gyroAvailable();
  #endif
}


bool initSensors()
{
  #ifdef CS_AG_PIN
    SPI.setSCK(14); //Because I still want to use the LED
    imu.settings.device.commInterface = IMU_MODE_SPI; // Set mode to SPI
    imu.settings.device.mAddress = CS_M_PIN; // Mag CS pin connected
    imu.settings.device.agAddress = CS_AG_PIN; // AG CS pin connected
  #else //I2C
    Wire.setClock(400000);
    imu.settings.device.commInterface = IMU_MODE_I2C;
  #endif
  
  imu.settings.accel.scale = 8;
  imu.settings.accel.sampleRate = 3; //~115Hz
  
  imu.settings.gyro.scale = 500;
  imu.settings.gyro.sampleRate = 3; //~115Hz

  imu.settings.mag.scale = 4;
  imu.settings.mag.sampleRate = 7; //80 hz
  imu.settings.mag.tempCompensationEnable = true;
  
  imu.settings.mag.XYPerformance = 2;
  imu.settings.mag.ZPerformance = 2;
  
  if (!imu.begin())
  {
    return false;
  }
  
  #ifdef INT1_PIN
    pinMode(INT1_PIN, INPUT_PULLUP);
    imu.configInt(XG_INT1, INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);
  #endif

  loadCalibration();

  return true;
}


void readSensors(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz)
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();
     
  ax = imu.calcAccel(imu.ax) - cal[0];
  ay = -imu.calcAccel(imu.ay) - cal[1];
  az = imu.calcAccel(imu.az) - cal[2];
  gx = imu.calcGyro(imu.gx) - cal[3];
  gy = -imu.calcGyro(imu.gy) - cal[4];
  gz = imu.calcGyro(imu.gz) - cal[5]; 
  float x = -(imu.calcMag(imu.mx) * 100) - cal[6];
  float y = -(imu.calcMag(imu.my) * 100) - cal[7];
  float z = (imu.calcMag(imu.mz) * 100) - cal[8];
  mx = x * cal[10] + y * cal[13] + z * cal[14];
  my = x * cal[13] + y * cal[11] + z * cal[15];
  mz = x * cal[14] + y * cal[15] + z * cal[12];

  gx = TO_RAD(gx);
  gy = TO_RAD(gy);
  gz = TO_RAD(gz);
}

void printSensorsRaw()
{
  int ax, ay, az;
  int gx, gy, gz;
  int mx, my, mz;
  
  imu.readGyro();
  imu.readAccel();
  imu.readMag();    

  ax = imu.calcAccel(imu.ax) / G_PER_COUNT;
  ay = -imu.calcAccel(imu.ay) / G_PER_COUNT;
  az = imu.calcAccel(imu.az) / G_PER_COUNT;
  gx = imu.calcGyro(imu.gx) / DEG_PER_SEC_PER_COUNT;
  gy = -imu.calcGyro(imu.gy) / DEG_PER_SEC_PER_COUNT;
  gz = imu.calcGyro(imu.gz) / DEG_PER_SEC_PER_COUNT;
  mx = -imu.calcMag(imu.mx) / (UT_PER_COUNT / 100.0f);
  my = -imu.calcMag(imu.my) / (UT_PER_COUNT / 100.0f);
  mz = imu.calcMag(imu.mz) / (UT_PER_COUNT / 100.0f);

  Serial.print("Raw:");
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(mx);
  Serial.print(',');
  Serial.print(my);
  Serial.print(',');
  Serial.print(mz);
  Serial.println();
}

void loadCalibration()
{
  unsigned char buf[CAL_SIZE];
  uint8_t i;
  uint16_t crc;
  
  for (i=0; i < CAL_SIZE; i++) {
    buf[i] = EEPROM.read(CAL_EEADDR + i);
  }
  crc = 0xFFFF;
  for (i=0; i < CAL_SIZE; i++) {
    crc = _crc16_update(crc, buf[i]);
  }
  if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
    memcpy(cal, buf+2, sizeof(cal));
  } else {
    memset(cal, 0, sizeof(cal));
    cal[9] = 50.0f;
  }
}

void doMagCalibration()
{  
  // get and print uncalibrated data
  printSensorsRaw();  

  
  loopcount = loopcount + 1;

  // check for incoming calibration
  receiveCalibration();

  // occasionally print calibration
  if (loopcount == 100) {
    Serial.print("Cal1:");
    for (int i=0; i<9; i++) {
      Serial.print(cal[i], 3);
      Serial.print(',');
    }
    Serial.println(cal[9], 3);
  }
  if (loopcount >= 200) {
    Serial.print("Cal2:");
    Serial.print(cal[10], 4); Serial.print(',');
    Serial.print(cal[13], 4); Serial.print(',');
    Serial.print(cal[14], 4); Serial.print(',');
    Serial.print(cal[13], 4); Serial.print(',');
    Serial.print(cal[11], 4); Serial.print(',');
    Serial.print(cal[15], 4); Serial.print(',');
    Serial.print(cal[14], 4); Serial.print(',');
    Serial.print(cal[15], 4); Serial.print(',');
    Serial.println(cal[12], 4);
    loopcount = 0;
  }
}

void receiveCalibration() {
  uint16_t crc;
  byte b, i, j;

  if (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < CAL_SIZE) {
      // full calibration message is 68 bytes
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < CAL_SIZE; i++) {
      crc = _crc16_update(crc, caldata[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      for (j=0; j < CAL_SIZE; j++) {
        EEPROM.write(CAL_EEADDR + j, caldata[j]);
      }
      memcpy(cal, caldata+2, sizeof(cal));
      return;
    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < CAL_SIZE - 1; i++) {
      if (caldata[i] == 117 && caldata[i+1] == 84) {
        // found possible start within data
        calcount = CAL_SIZE - i;
        memmove(caldata, caldata + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (caldata[CAL_SIZE - 1] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}

