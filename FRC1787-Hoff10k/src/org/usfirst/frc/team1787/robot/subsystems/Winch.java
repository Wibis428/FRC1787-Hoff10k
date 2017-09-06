package org.usfirst.frc.team1787.robot.subsystems;

import com.ctre.CANTalon;

public class Winch {
  
  // Talons
  private final int WINCH_TALON_ID = 4;
  private final int ADDITIONAL_WINCH_TALON_ID = 10;
  private CANTalon winchMotor = new CANTalon(WINCH_TALON_ID);
  private CANTalon winchMotor2 = new CANTalon(ADDITIONAL_WINCH_TALON_ID);
  
  private final double DEFAULT_CLIMB_SPEED = 1.0;
  private final double DEFAULT_DECEND_SPEED = -0.5;
  
  // Singleton Instance
  private static final Winch instance = new Winch();
  
  private Winch() {
    // Intentionally left blank. No initialization code required.
  }

  public void climb() {
    winchMotor.set(-DEFAULT_CLIMB_SPEED);
    winchMotor2.set(DEFAULT_CLIMB_SPEED);
  }

  public void decend() {
    winchMotor.set(-DEFAULT_DECEND_SPEED);
    winchMotor2.set(DEFAULT_DECEND_SPEED);
  }

  public void stop() {
    winchMotor.set(0);
    winchMotor2.set(0);
  }

  /** 
   * @param winchSpeed Positive values for going up, negative values for going down.
   */
  public void manualControl(double value) {
    winchMotor.set(-value);
    winchMotor2.set(value);
  }
  
  public static Winch getInstance() {
    return instance;
  }
}
