package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.utils.UnitConverter;
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
  private final double METERS_PER_PULSE = UnitConverter.inchesToMeters(0.01249846);
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
    leftEncoder.setDistancePerPulse(METERS_PER_PULSE);
    rightEncoder.setDistancePerPulse(METERS_PER_PULSE);
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
  
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }
  
  public Encoder getRightEncoder() {
    return rightEncoder;
  }
  
  /**
   * @param meters Whether or not to return the speed in m/s or in/s
   */
  public double getAvgSpeed() {
    return (leftEncoder.getRate() + rightEncoder.getRate()) / 2.0;
  }
  
  // Other Methods

  public void publishDataToSmartDash() {
    SmartDashboard.putNumber("Average Speed (m/s)", getAvgSpeed());
    SmartDashboard.putBoolean("Low Gear", isInLowGear());
    SmartDashboard.putNumber("Left Drive Encoder Ticks", leftEncoder.getRaw());
    SmartDashboard.putNumber("Right Drive Encoder Ticks", rightEncoder.getRaw());
    SmartDashboard.putNumber("Left Drive Encoder Distance (m)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Drive Encoder Distance (m)", leftEncoder.getDistance());
  }
  
  public static DriveTrain getInstance() {
    return instance;
  }
}
