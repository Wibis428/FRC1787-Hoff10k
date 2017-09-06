package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.vision.Target;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
  
  // Driving Talons
  private final int FRONT_RIGHT_TALON_ID = 8;
  private final int REAR_RIGHT_TALON_ID = 9;
  private final int FRONT_LEFT_TALON_ID = 6;
  private final int REAR_LEFT_TALON_ID = 7;
  private CANTalon frontLeftMotor = new CANTalon(FRONT_LEFT_TALON_ID);
  private CANTalon rearLeftMotor = new CANTalon(REAR_LEFT_TALON_ID);
  private CANTalon frontRightMotor = new CANTalon(FRONT_RIGHT_TALON_ID);
  private CANTalon rearRightMotor = new CANTalon(REAR_RIGHT_TALON_ID);
  private RobotDrive driveController = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  
  // Encoders (one for each side of the drivetrain)
  private final int LEFT_ENCODER_A_CHANNEL = 2;
  private final int LEFT_ENCODER_B_CHANNEL = 3;
  private final int RIGHT_ENCODER_A_CHANNEL = 0;
  private final int RIGHT_ENCODER_B_CHANNEL =1;
  // determined through testing
  private final double INCHES_PER_PULSE = 0.01249846;
  private Encoder leftEncoder = new Encoder(LEFT_ENCODER_A_CHANNEL, LEFT_ENCODER_B_CHANNEL);
  private Encoder rightEncoder = new Encoder(RIGHT_ENCODER_A_CHANNEL, RIGHT_ENCODER_B_CHANNEL);

  // Gear Shifter (pneumatic shifter controlled by a solenoid)
  private final int SOLENOID_ID = 0;
  /* The boolean value that corresponds to each gear was determined through testing.
   * These booleans indicate the correct value to use when calling "solenoid.set()". */
  private final boolean HIGH_GEAR = false;
  private final boolean LOW_GEAR = true;
  private Solenoid gearShifter = new Solenoid(SOLENOID_ID);
  
  // Singleton Instance
  private static final DriveTrain instance = new DriveTrain();

  private DriveTrain() {
    leftEncoder.setDistancePerPulse(INCHES_PER_PULSE);
    rightEncoder.setDistancePerPulse(INCHES_PER_PULSE);
  }
  
  // Drive Train Related Methods
  
  public void regularArcadeDrive(Joystick stick) {
    driveController.arcadeDrive(stick);
  }

  public void reverseArcadeDrive(Joystick stick) {
    driveController.arcadeDrive(-stick.getY(), stick.getX());
  }
  
  public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
    driveController.setLeftRightMotorOutputs(leftOutput, rightOutput);
  }
  
  public void stop() {
    driveController.setLeftRightMotorOutputs(0, 0);
  }
  
  // Gear Shifter Related Methods
  
  public void setLowGear() {
    if (!isInLowGear()) {
      gearShifter.set(LOW_GEAR);
    }
  }
  
  public void setHighGear() {
    if (isInLowGear()) {
      gearShifter.set(HIGH_GEAR);
    }
  }
  
  public boolean isInLowGear() {
    return gearShifter.get();
  }

  public void toggleGear() {
    gearShifter.set(!gearShifter.get());
  }
  
  // Encoder Related Methods
  
  public void zeroSensors() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  
  public double getLeftVelocity(boolean meters) {
    return meters ? Target.getMeters(leftEncoder.getRate()) : leftEncoder.getRate();
  }
  
  public double getLeftDistance(boolean meters) {
    return meters ? Target.getMeters(leftEncoder.getDistance()) : leftEncoder.getDistance();
  }
  
  public double getRightVelocity(boolean meters) {
    return meters ? Target.getMeters(rightEncoder.getRate()) : leftEncoder.getRate();
  }
  
  public double getRightDistance(boolean meters) {
    return meters ? Target.getMeters(rightEncoder.getDistance()) : rightEncoder.getDistance();
  }
  
  /**
   * @param meters Whether or not to return the speed in m/s or in/s
   */
  public double getAvgSpeed(boolean meters) {
    return (getLeftVelocity(meters) + getRightVelocity(meters)) / 2.0;
  }

  public void publishDataToSmartDash() {
    SmartDashboard.putNumber("Average Speed (m/s)", getAvgSpeed(true));
    SmartDashboard.putBoolean("Low Gear", isInLowGear());
    SmartDashboard.putNumber("Left Drive Encoder Ticks", leftEncoder.get());
    SmartDashboard.putNumber("Right Drive Encoder Ticks", rightEncoder.get());
    SmartDashboard.putNumber("Left Drive Encoder Distance (m)", this.getLeftDistance(true));
    SmartDashboard.putNumber("Right Drive Encoder Distance (m)", this.getRightDistance(true));
  }
  
  public static DriveTrain getInstance() {
    return instance;
  }
}
