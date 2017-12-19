package org.usfirst.frc.team1787.robot.utils;

import com.ctre.CANTalon;

public class TalonConfigurer {
  
  /**
   * Configs the given talon as follows
   * 1) changes control mode to Voltage control, 
   * which allows you to specify the voltage that the talon should apply to the motor.
   * 
   * 2) enables break mode
   * 
   * 3) sets the maximum voltage ramp rate
   * 
   * 4) sets a current limit
   * @param talon
   */
  public static void configTalon(CANTalon talon) {
    talon.enableBrakeMode(true);
    
    // setting a ramp rate of 0 indicates that no max ramp rate will be enforced.
    talon.setVoltageRampRate(0);
    
    talon.setCurrentLimit(0);
    talon.EnableCurrentLimit(false);
    
    //talon.changeControlMode(CANTalon.TalonControlMode.Voltage);
    // Note: was gonna try Voltage control mode, as it appeared it would be more consistent, 
    // but the wpilib PIDController
    // requires PercentVbus, which is the default control mode of the talon.
    // It takes in a number between -1 and 1 and sets the voltage to be that percentage.
    talon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    talon.set(0);
  }
  
  /* Random Notes on CANTalon */
  /* talon is constructed with default update rate of 10ms
   * if a different update rate is desired, it can be set as a 2nd 
   * parameter to the constructor. The given # will be the update rate in ms.
   * note that increasing this rate will increase bandwith. leaving it at the deafault is probably best
   * for now until we have time to research / test the consequences of chaning it.
   */
}
