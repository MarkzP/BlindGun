
#define TRIGGER_PIN     0
#define BUTTON1_PIN     1
#define BUTTON2_PIN     5

// If you need more buttons, follow the logic of button1 & 2 as a template

#define CALIB_HOLD_COUNT 250
#define SWITCH_MODE_COUNT 500
int buttonHoldCount = 0;

bool trigger = false;
bool button1 = false;
bool button2 = false;


float oldYawZero = 0.0f;
float oldPitchZero = 0.0f;

void initButtons()
{
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
}


void checkButtons()
{
  // This will check actual pins & generate events on change.
  // It will also do screen range calibration when the trigger is held down for some time
  // For those wondering, no need to debounce - most MAME games already implement some form of control debouncing anyways, it would introduce more lag
  
  bool triggerPressed = !digitalRead(TRIGGER_PIN);
  bool button1Pressed = !digitalRead(BUTTON1_PIN);
  bool button2Pressed = !digitalRead(BUTTON2_PIN);

  if (triggerPressed != trigger)
  {
    setButtonState(1, triggerPressed);
    trigger = triggerPressed;
  }

  if (button1Pressed != button1)
  {
    setButtonState(2, button1Pressed);
    button1 = button1Pressed;
  }

  if (button2Pressed != button2)
  {
    setButtonState(3, button2Pressed);
    button2 = button2Pressed;
  }

  if (triggerPressed)
  {    
    buttonHoldCount++; 

    if (!button1Pressed && buttonHoldCount == CALIB_HOLD_COUNT)
    {
      //calibrate screen area et Upper left corner 
      digitalWrite(LED_BUILTIN, HIGH);
      calibrating = true;
        
      oldYawZero = yawZero;
      oldPitchZero = pitchZero;

      yawZero = yaw;
      pitchZero = pitch;
    }

    if (button1Pressed && buttonHoldCount == SWITCH_MODE_COUNT)
    {
      if (useMouse)
      {
        Mouse.release(MOUSE_ALL);
      }
      else
      {
        Joystick.button(1, false);
        Joystick.button(2, false);
        Joystick.button(3, false);
      }

      useMouse = !useMouse;
      
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  else
  {
    if (calibrating)
    {
      //Button released after being held: get Lower right corner
      if (relativeYaw > 0 && relativePitch > 0)
      {
        yawRange = relativeYaw;
        pitchRange = relativePitch;

        savePitchYaw();
      }
      else
      {
        yawZero = oldYawZero;
        pitchZero = oldPitchZero;
      }

      calibrating = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    buttonHoldCount = 0;  
  }
}

