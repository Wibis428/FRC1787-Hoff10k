package org.usfirst.frc.team1787.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

/**
 * An extension of the included Joystick class that 
 * provides additional functionality.
 */
public class CustomJoystick extends Joystick {
  
  //Joystick Axes for reference (these values determined through testing)
  public static final int JOYSTICK_SLIDER_AXIS = 3;
  public static final int JOYSTICK_ROTATE_AXIS = 2;
  
  private boolean[] buttonWasPressed;
  
  public CustomJoystick(int port) {
    super(port);
    // The getButtonCount() method doesn't seem to be working, so a default of 20 is now used
    //buttonWasPressed = new boolean[this.getButtonCount()+1];
    buttonWasPressed = new boolean[20];
  }
  
  /**
   * Returns true if the given button is pressed,
   * but will not continue to return true if the button is held down.
   * @param button
   * @return
   */
  public boolean getSinglePress(int button) {
    if (this.getRawButton(button) && !buttonWasPressed[button]) {
      buttonWasPressed[button] = true;
      return true;
    } else if (!this.getRawButton(button) && buttonWasPressed[button]) {
      buttonWasPressed[button] = false;
    }
    return false;
  }
  
  /**
   * @return the value of the slider.
   */
  public double getSlider() {
    return this.getRawAxis(JOYSTICK_SLIDER_AXIS);
  }
  
  /**
   * @return the value of the rotation of the stick.
   */
  public double getYaw() {
    return this.getRawAxis(JOYSTICK_ROTATE_AXIS);
  }
}
