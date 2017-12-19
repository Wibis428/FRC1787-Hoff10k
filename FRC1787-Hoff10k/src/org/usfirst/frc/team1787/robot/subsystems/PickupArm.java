package org.usfirst.frc.team1787.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PickupArm {

  // Arm (The pistons that move the arm are controlled by a double solenoid)
  private final int SOLENOID_FORWARD_CHANNEL = 1;
  private final int SOLENOID_REVERSE_CHANNEL = 2;
  private DoubleSolenoid pickupArmSolenoid = new DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, 
                                                                SOLENOID_REVERSE_CHANNEL);
  // these values determined through testing
  public final DoubleSolenoid.Value DEPLOY = DoubleSolenoid.Value.kReverse;
  public final DoubleSolenoid.Value RETRACT = DoubleSolenoid.Value.kForward;
  
  // Pickup Wheels (Controlled by a talon)
  private final int PICKUP_WHEELS_TALON_ID = 3;
  public final double DEFAULT_INTAKE_SPEED = -0.8;
  private CANTalon pickupWheelsMotor = new CANTalon(PICKUP_WHEELS_TALON_ID);
  
  // Singleton Instance
  private static final PickupArm instance = new PickupArm();

  private PickupArm() {
    // Intentionally left blank. No initialization required.
  }
  
  /**
   * @param desiredState intended to be one of either pickupArm.DEPLOY
   * or pickupArm.RETRACT
   */
  public void moveArm(DoubleSolenoid.Value desiredState) {
    pickupArmSolenoid.set(desiredState);
  }
  
  public void spinIntake(double moveValue) {
    pickupWheelsMotor.set(moveValue);
  }
  
  public void stop() {
    pickupWheelsMotor.set(0);
  }
  
  public static PickupArm getInstance() {
    return instance;
  }
}
