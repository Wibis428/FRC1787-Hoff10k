package org.usfirst.frc.team1787.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PickupArm {

  // Arm (The pistons that move the arm are controlled by a double solenoid)
  private final int SOLENOID_FORWARD_CHANNEL = 1;
  private final int SOLENOID_REVERSE_CHANNEL = 2;
  private DoubleSolenoid pickupArmSolenoid = new DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, 
                                                                SOLENOID_REVERSE_CHANNEL);
  
  // Pickup Wheels (Controlled by a talon)
  private final int PICKUP_WHEELS_TALON_ID = 3;
  private final double DEFAULT_INTAKE_SPEED = -0.8;
  private CANTalon pickupWheelsMotor = new CANTalon(PICKUP_WHEELS_TALON_ID);
  
  // Singleton Instance
  private static final PickupArm instance = new PickupArm();

  private PickupArm() {
    // Intentionally left blank. No initialization required.
  }

  public void deploy() {
    pickupArmSolenoid.set(DoubleSolenoid.Value.kReverse);
    /* The appropriate value to make the arm 
     * deploy was determined through testing
     */
  }

  public void retract() {
    pickupArmSolenoid.set(DoubleSolenoid.Value.kForward);
    /* The appropriate value to make the arm
     * retract was determined through testing.
     */
  }

  public void intake() {
    pickupWheelsMotor.set(DEFAULT_INTAKE_SPEED);
  }

  public void expell() {
    pickupWheelsMotor.set(-DEFAULT_INTAKE_SPEED);
  }

  public void stopPickupWheels() {
    pickupWheelsMotor.set(0);
  }
  
  public void manuallyControlIntake(double value) {
    pickupWheelsMotor.set(value);
  }
  
  public static PickupArm getInstance() {
    return instance;
  }
}
