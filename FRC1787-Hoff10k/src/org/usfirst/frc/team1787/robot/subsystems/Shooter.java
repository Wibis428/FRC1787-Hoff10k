package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.vision.ImageProcessor;
import org.usfirst.frc.team1787.robot.vision.Target;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The shooter class is composed of the turret, the flywheel, and the feeder.
 * This class serves to coordinate those mechanisms.
 */
public class Shooter {

  // Sub-Mechanisms
  private Turret turret = Turret.getInstance();
  private Flywheel flywheel = Flywheel.getInstance();
  private Feeder feeder = Feeder.getInstance();
  
  // Vision
  private ImageProcessor imgProcessor = ImageProcessor.getInstance();
  
  // Singleton Instance
  private static final Shooter instance = new Shooter();

  private Shooter() {
    // Intentionally left blank; no initialization needed.
  }
  
  public void enablePIDControllers() {
    turret.getPIDController().enable();
    flywheel.getPIDController().enable();
  }
  
  public boolean pidIsEnabled() {
    return turret.getPIDController().isEnabled() ||
           flywheel.getPIDController().isEnabled();
  }
  
  public void fullAutoShooting() {
    trackTarget();
    double horizontalDistanceToTarget = imgProcessor.getCurrentTarget().getHorizontalDistance();
    double verticalDistanceToTarget = Target.TURRET_TO_TARGET_VERTICAL_DISTANCE;
    flywheel.setCalculatedSetpoint(horizontalDistanceToTarget, verticalDistanceToTarget);
    if (turret.getPIDController().onTarget() && flywheel.getPIDController().onTarget()) {
      feeder.spin(feeder.DEFAULT_FEEDER_SPEED);
    } else {
      feeder.stop();
    }
  }
  
  public void trackTarget() {
    imgProcessor.runVisionProcessing();
    turret.getPIDController().setRelativeSetpoint(imgProcessor.getCurrentTarget().getErrorInDegreesX());
  }
  
  public void zeroSensors() {
    turret.zeroSensors();
    flywheel.zeroSensors();
  }
  
  public void manualControl(double turretValue, double flywheelValue, double feederValue) {
    turret.manualControl(turretValue);
    flywheel.manualControl(flywheelValue);
    feeder.spin(feederValue);
  }
  
  public void manualControl(Joystick stick) {
    turret.manualControl(stick.getX());
    flywheel.manualControl(stick.getY());
    if (stick.getTrigger()) {
      feeder.spin(feeder.DEFAULT_FEEDER_SPEED);
    } else {
      feeder.stop();
    }
  }
  
  public void stop() {
    turret.stop();
    flywheel.stop();
    feeder.stop();
  }
  
  public void publishDataToSmartDash() {
    turret.publishDataToSmartDash();
    flywheel.publishDataToSmartDash();
  }
  
  public static Shooter getInstance() {
    return instance;
  }
}
