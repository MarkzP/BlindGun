
#define TRIGGER_PIN     0
#define BUTTON1_PIN     1
#define BUTTON2_PIN     5

#define HOLD_COUNT 250
int buttonHoldCount = 0;

bool trigger = false;
bool button1 = false;
bool button2 = false;

float oldYawZero = 1.0f;
float oldPitchZero = 0.0f;

void initButtons()
{
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
}


void checkButtons()
{
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

    if (buttonHoldCount == HOLD_COUNT)
    {
      //calibrate screen area et Upper left corner 
      digitalWrite(LED_BUILTIN, HIGH);
      calibrating = true;
        
      oldYawZero = yawZero;
      oldPitchZero = pitchZero;

      yawZero = yaw;
      pitchZero = pitch;
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

