
#include <EEPROM.h>
#include "DCM.h"

#define EADR 0x00
#define eepr_magic 0xF00C

DCM dcm;

void loadPitchYaw()
{
  uint32_t eepr_test;
  
  EEPROM.get(EADR + 0, eepr_test);
  if (eepr_test == eepr_magic)
  {
    EEPROM.get(EADR + 4, yawZero);
    EEPROM.get(EADR + 8, pitchZero);
    EEPROM.get(EADR + 12, yawRange);
    EEPROM.get(EADR + 16, pitchRange);
  }
}

void savePitchYaw()
{
  EEPROM.put(EADR + 4, yawZero);
  EEPROM.put(EADR + 8, pitchZero);
  EEPROM.put(EADR + 12, yawRange);
  EEPROM.put(EADR + 16, pitchRange);

  EEPROM.put(EADR + 0, eepr_magic);
}

void initOrientation()
{
  loadPitchYaw();
}

void updateOrientation()
{
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  readSensors(ax, ay, az, gx, gy, gz, mx, my, mz);

  //Update Orientation Filter
  dcm.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  yaw = -dcm.yaw;
  pitch = dcm.pitch;
  roll = dcm.roll;

  //Compute angles relative to screen area
  relativePitch = pitch - pitchZero;
  relativeYaw = yaw - yawZero;

  //Complete the circle
  if (relativeYaw < 0)
  {
    relativeYaw += 2 * PI;
  }
  if (relativeYaw > PI)
  {
    relativeYaw = (-2 * PI) + relativeYaw;
  }

  pos_x = relativeYaw / yawRange;
  pos_y = relativePitch / pitchRange;
}

