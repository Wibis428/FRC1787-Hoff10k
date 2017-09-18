package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.utils.CustomPIDController;
import org.usfirst.frc.team1787.robot.vision.Target;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel {
  
  // Talon
  private final int TURRET_FLYWHEEL_TALON_ID = 5;
  private CANTalon flywheelMotor = new CANTalon(TURRET_FLYWHEEL_TALON_ID);
  
  // Encoder Constants
  private final int FLYWHEEL_ENCODER_A_CHANNEL = 6;
  private final int FLYWHEEL_ENCODER_B_CHANNEL = 7;
  // 2048 encoder ticks per encoder revolution, and the encoder is mounted on the same axle as the flywheel
  private final double FLYWHEEL_ENCODER_REVOLUTIONS_PER_PULSE = 1.0 / 2048;
  private Encoder flywheelEncoder = new Encoder(FLYWHEEL_ENCODER_A_CHANNEL, FLYWHEEL_ENCODER_B_CHANNEL);

  // PID Control Loop Gains / Preferences
  private final double FLYWHEEL_PID_CONTROLLER_KP = 0;
  private final double FLYWHEEL_PID_CONTROLLER_KI = 0;
  private final double FLYWHEEL_PID_CONTROLLER_KD = 0;
  private final double FLYWHEEL_PID_ABSOLUTE_TOLERENCE_IN_REVOLUTIONS_PER_SECOND = 0;
  private CustomPIDController flywheelController = new CustomPIDController(FLYWHEEL_PID_CONTROLLER_KP, 
                                                               FLYWHEEL_PID_CONTROLLER_KI, 
                                                               FLYWHEEL_PID_CONTROLLER_KD,
                                                               1.0/80, flywheelEncoder, flywheelMotor, 
                                                               PIDController.kDefaultPeriod);

  // Geometric Constants (in meters)
  //(X inches) * (0.0254 meters / inch)
  private final double FLYWHEEL_RADIUS = 1 * 0.0254;
  private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
  private final double EXIT_ANGLE_DEGREES = 1;
  private final double EXIT_ANGLE_RADIANS = Math.toRadians(EXIT_ANGLE_DEGREES);
  
  // Singleton Instance
  private static final Flywheel instance = new Flywheel();
  
  private Flywheel() {
    // Configure Talon
    flywheelMotor.enableBrakeMode(false);

    // Configure Encoder
    flywheelEncoder.setPIDSourceType(PIDSourceType.kRate);
    flywheelEncoder.setDistancePerPulse(FLYWHEEL_ENCODER_REVOLUTIONS_PER_PULSE);
    flywheelEncoder.setReverseDirection(true);
    
    // Configure PID Controller
    flywheelController.setAbsoluteTolerance(FLYWHEEL_PID_ABSOLUTE_TOLERENCE_IN_REVOLUTIONS_PER_SECOND);
  }
  
  public CustomPIDController getPIDController() {
    return flywheelController;
  }
  
  /**
   * Sets the flywheel to the appropriate speed for the given distance
   * @param distance The distance to the target in meters
   */
  public void setCalculatedSetpoint(double distance) {
    /* TO DO: figure out how to determine the appropriate
     * flywheel speed for a given distance. 
     * 
     * Because it was determined from standard kinematic equations, 
     * the method that's commented out below should work in a physics friendly world 
     * (i.e. no air resistance, no holes or spin on the ball, the ball rolls without slipping in the turret), 
     * but that's not the world we live in. Still, it probably wouldn't be a bad idea to try it out 
     * and see how close it is. If you're going to try it out, make sure to comment out the last line of this method
     * first. Otherwise, the calculation will be overridden.
     * */
    
    
    double numeratorSquared = (1.0 / 2) * (-9.81) * Math.pow(distance / Math.cos(EXIT_ANGLE_RADIANS), 2);
    double denominatorSquared = Target.TURRET_TO_TARGET_VERTICAL_DISTANCE_METERS - (distance * Math.tan(EXIT_ANGLE_RADIANS));
    double requiredExitVelocity = Math.sqrt(numeratorSquared / denominatorSquared);
    
    // (Meters / Second) * (1 Revolution / CIRCUMFERENCE meters) = Revolutions / Second
    // Theoretically, the translational velocity of the ball will be half of the tangential velocity of the edge of the flywheel.
    // Therefore, for the ball to achieve the requiredExitVelocity, the edge of the flywheel must be moving twice as fast.
    double calculatedSetpoint = (requiredExitVelocity * 2 * (1.0 / FLYWHEEL_CIRCUMFERENCE));
    flywheelController.setSetpoint(calculatedSetpoint);
    
    
    flywheelController.setSetpoint(distance);
  }
  
  public void zeroSensors() {
    flywheelEncoder.reset();
  }
  
  public void manualControl(double value) {
    if (flywheelController.isEnabled()) {
      flywheelController.reset();
    }
    flywheelMotor.set(value);
  }
  
  public void stop() {
    manualControl(0);
  }

  public void publishDataToSmartDash() {
    SmartDashboard.putBoolean("Flywheel PID Enabled", flywheelController.isEnabled());
    SmartDashboard.putNumber("Flywheel Encoder Ticks", flywheelEncoder.get());
    SmartDashboard.putNumber("flywheelRPS", flywheelEncoder.getRate());
    SmartDashboard.putNumber("flywheelError", flywheelController.getError());
    SmartDashboard.putNumber("flywheelOutputVoltage", flywheelController.get());
    SmartDashboard.putBoolean("Flywheel On Target", flywheelController.onTarget());
  }
  
  public static Flywheel getInstance() {
    return instance;
  }
}