package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.vision.CameraController;

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
  private CameraController camController = CameraController.getInstance();
  
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
    flywheel.setCalculatedSetpoint(camController.getCurrentTarget().getDistance());
    if (turret.getPIDController().onTarget() && flywheel.getPIDController().onTarget()) {
      feeder.feed();
    } else {
      feeder.stop();
    }
  }
  
  public void trackTarget() {
    camController.runVisionProcessing();
    turret.getPIDController().setRelativeSetpoint(camController.getCurrentTarget().getErrorInDegreesX());
  }
  
  public void zeroSensors() {
    turret.zeroSensors();
    flywheel.zeroSensors();
  }
  
  public void manualControl(double turretValue, double flywheelValue, double feederValue) {
    turret.manualControl(turretValue);
    flywheel.manualControl(flywheelValue);
    feeder.manualControl(feederValue);
  }
  
  public void manualControl(Joystick stick) {
    turret.manualControl(stick.getX());
    flywheel.manualControl(stick.getY());
    if (stick.getTrigger()) {
      feeder.feed();
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
