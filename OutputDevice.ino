

#define JOY_MAX    1023
#define JOY_MID    511


#define MOUSE_MAX 4096

bool onscreen = false;
bool wasOnscreen = false;

void initOutputDevice()
{
  Joystick.useManualSend(true);
  
  Joystick.X(JOY_MID);
  Joystick.Y(JOY_MID);
  Joystick.Z(JOY_MID);
  Joystick.sliderLeft(JOY_MID);
  Joystick.sliderRight(JOY_MID);
  Joystick.Zrotate(JOY_MID);
  Joystick.hat(-1);

  Mouse.screenSize(MOUSE_MAX, MOUSE_MAX);
}

void setButtonState(uint8_t button, bool val)
{
  if (useMouse)
  {
    if (val)
    {
      switch (button)
      {
        case 1: Mouse.press(MOUSE_LEFT); break;
        case 2: Mouse.press(MOUSE_RIGHT); break;
        case 3: Mouse.press(MOUSE_MIDDLE); break;
      }
    }
    else
    {
      switch (button)
      {
        case 1: Mouse.release(MOUSE_LEFT); break;
        case 2: Mouse.release(MOUSE_RIGHT); break;
        case 3: Mouse.release(MOUSE_MIDDLE); break;
      } 
    }
  }
  else
  {
    Joystick.button(button, val);
  }
}

void sendPosition()
{
  onscreen = pos_x >= 0.0f && pos_x < 1.0f && pos_y >= 0.0f && pos_y < 1.0f;

  if (!onscreen)
  {
    pos_x = 1.0f;
    pos_y = 1.0f;
  }
  
  if (useMouse)
  {
    if (onscreen || wasOnscreen)
    {
      Mouse.moveTo(pos_x * MOUSE_MAX, pos_y * MOUSE_MAX);
    }
  }
  else
  {
    Joystick.X(pos_x * JOY_MAX);
    Joystick.Y(pos_y * JOY_MAX);
    Joystick.send_now();    
  }

  wasOnscreen = onscreen;
}

