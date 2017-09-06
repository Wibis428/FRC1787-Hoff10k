package org.usfirst.frc.team1787.robot.subsystems;

import com.ctre.CANTalon;

public class Feeder {
  
  // Talons
  private final int TURRET_FEEDER_TALON_ID = 2;
  private final double DEFAULT_FEEDER_SPEED = -0.42;
  private CANTalon feederMotor = new CANTalon(TURRET_FEEDER_TALON_ID);
  
  // Singleton Instance
  private static final Feeder instance = new Feeder();

  private Feeder() {
    // Intentionally left blank. No initialization code required.
  }

  public void feed() {
    feederMotor.set(DEFAULT_FEEDER_SPEED);
  }

  public void reverse() {
    feederMotor.set(-DEFAULT_FEEDER_SPEED);
  }

  public void stop() {
    feederMotor.set(0);
  }
  
  public void manualControl(double value) {
    feederMotor.set(value);
  }
  
  public static Feeder getInstance() {
    return instance;
  }
}
