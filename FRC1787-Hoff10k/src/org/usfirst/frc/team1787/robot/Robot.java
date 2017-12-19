package org.usfirst.frc.team1787.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.usfirst.frc.team1787.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1787.robot.subsystems.Flywheel;
import org.usfirst.frc.team1787.robot.subsystems.PickupArm;
import org.usfirst.frc.team1787.robot.subsystems.Shooter;
import org.usfirst.frc.team1787.robot.subsystems.Turret;
import org.usfirst.frc.team1787.robot.subsystems.Winch;
import org.usfirst.frc.team1787.robot.utils.CustomJoystick;
import org.usfirst.frc.team1787.robot.vision.CameraController;
import org.usfirst.frc.team1787.robot.vision.ImageProcessor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
  // Don't Ask
  protected int farfar37;
  
  // Controls
  private final int RIGHT_JOYSTICK_ID = 0;
  private final int LEFT_JOYSTICK_ID = 1;
  private CustomJoystick rightStick = new CustomJoystick(RIGHT_JOYSTICK_ID);
  private CustomJoystick leftStick = new CustomJoystick(LEFT_JOYSTICK_ID);
  
  // Button Map
  private final int DEPLOY_ARM_BUTTON = 3;
  private final int RETRACT_ARM_BUTTON = 4;
  private final int INTAKE_BUTTON = 1;
  private final int EXPELL_BUTTON = 2;
  
  private final int WINCH_CLIMB_BUTTON = 8;
  //private final int WINCH_DESCEND_BUTTON = 7;
  /* The ratchet on the winch motor prevents the winch from rotating in the opposite direction,
   * so the WINCH_DESCEND_BUTTON is never actually being used. However, it's value remains
   * here for if we ever decide to remove the ratchet. */
  
  // keeps track of the current "mode" of the shooter
  // current options include mode 0 (manual control), and mode 1 (full auto shooting)
  private int shooterControlMode = 0;
  private final int TOGGLE_SHOOTER_CONTROL_BUTTON = 2;
  private final int TOGGLE_CAM_BUTTON = 10;
  
  // Testing Mode Stuff
  private boolean tuningModeActive = false;
  private final int TOGGLE_TUNING_MODE_BUTTON = -1;
  
  // if tuning mode is active, this variable determines what
  // exactly is being tuned
  private int tuningMode = 0;
  private final int CYCLE_THROUGH_TUNING_MODES_BUTTON = -1;
  
  // Instances of Subsystems
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private PickupArm pickupArm = PickupArm.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Winch winch = Winch.getInstance();
  private CameraController camController = CameraController.getInstance();
  private ImageProcessor imgProcessor = ImageProcessor.getInstance();
  
  // These are only used for tuning
  private Flywheel flywheel = Flywheel.getInstance();
  private Turret turret = Turret.getInstance();
  
  // Preferences (used to get values from the smart dash)
  Preferences prefs = Preferences.getInstance();
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
  }

  /**
   * This function is run once upon entering autonomous.
   */
  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  /**
   * This function is run once upon entering teleop mode.
   */
  public void teleopInit() {
    SmartDashboard.putBoolean("Tuning Mode Active", tuningModeActive);
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    // Driving
    if (rightStick.getMagnitude() > leftStick.getMagnitude()) {
      driveTrain.arcadeDrive(rightStick.getY(), rightStick.getX());
    } else {
      /* Use of the left stick is currently reserved for the shooter. */
      //driveTrain.arcadeDrive(-1*rightStick.getY(), rightStick.getX());
    }
    
    // Gear Shifter
    if (rightStick.getSlider() < 0) {
      driveTrain.setGear(driveTrain.HIGH_GEAR);
    } else {
      driveTrain.setGear(driveTrain.LOW_GEAR);
    }
    driveTrain.publishDataToSmartDash();
    
    // Pickup Arm
    if (rightStick.getRawButton(DEPLOY_ARM_BUTTON)) {
      pickupArm.moveArm(pickupArm.DEPLOY);
    } else if (rightStick.getRawButton(RETRACT_ARM_BUTTON)) {
      pickupArm.moveArm(pickupArm.RETRACT);
    }
    
    // Pickup Wheels
    if (rightStick.getRawButton(INTAKE_BUTTON)) {
      pickupArm.spinIntake(pickupArm.DEFAULT_INTAKE_SPEED);
    } else if (rightStick.getRawButton(EXPELL_BUTTON)) {
      pickupArm.spinIntake(-1 * pickupArm.DEFAULT_INTAKE_SPEED);
    } else {
      pickupArm.spinIntake(0);
    }
    
    // Winch
    if (rightStick.getRawButton(WINCH_CLIMB_BUTTON)) {
      winch.spin(winch.DEFAULT_CLIMB_SPEED);
    } else {
      winch.stop();
    }
    
    // Tuning Mode
    if (leftStick.getSinglePress(TOGGLE_TUNING_MODE_BUTTON)) {
      tuningModeActive = !tuningModeActive;
      SmartDashboard.putBoolean("Tuning Mode Active", tuningModeActive);
      shooter.stop();
    }
    
    if (tuningModeActive) {
      runTuningCode();
      return;
    }
    
    // Shooter
    if (leftStick.getSinglePress(TOGGLE_SHOOTER_CONTROL_BUTTON)) {
      shooter.stop();
      shooterControlMode = (shooterControlMode + 1) % 2;
    }
    
    if (shooterControlMode == 0) {
      // Shooter Mode 0 = Manual Control
      shooter.manualControl(leftStick);
    } else if (shooterControlMode == 1) {
      // Shooter Mode 1 = Full Auto Shooting
      if (!shooter.pidIsEnabled()) {
        shooter.enablePIDControllers();
      }
      shooter.fullAutoShooting();
    }
    shooter.publishDataToSmartDash();
    
    // Cams (note that most img processing code is already called by shooter.fullAutoShooting())
    if (rightStick.getSinglePress(TOGGLE_CAM_BUTTON)) {
      camController.toggleCamStream();
    }
    imgProcessor.publishDataToSmartDash();
  }
  
  public void runTuningCode() {
    if (leftStick.getSinglePress(CYCLE_THROUGH_TUNING_MODES_BUTTON)) {
      tuningMode = (tuningMode + 1) % 4;
      shooter.stop();
    }
    
    if (tuningMode == 0) {
      // Tuning Mode 0 = Turret PID Testing
      if (!shooter.pidIsEnabled()) {
        double turretP = prefs.getDouble("turretP", 0);
        double turretI = prefs.getDouble("turretI", 0);
        double turretD = prefs.getDouble("turretD", 0);
        turret.getPIDController().setPID(turretP, turretI, turretD);
        
        double turretTolerance = prefs.getDouble("turretDegreesTolerance", 0);
        turret.getPIDController().setAbsoluteTolerance(turretTolerance);
        
        turret.getPIDController().enable();
      }
      shooter.trackTarget();
    } else if (tuningMode == 1) {
      // Tuning Mode 1 = Flywheel PID Testing
      if (!shooter.pidIsEnabled()) {
        double flywheelP = prefs.getDouble("flywheelP", 0);
        double flywheelI = prefs.getDouble("flywheelI", 0);
        double flywheelD = prefs.getDouble("flywheelD", 0);
        flywheel.getPIDController().setPID(flywheelP, flywheelI, flywheelD);
        
        double flywheelTolerance = prefs.getDouble("flywheelRPSTolerance", 0);
        flywheel.getPIDController().setAbsoluteTolerance(flywheelTolerance);
        
        flywheel.getPIDController().enable();
      }
      double flywheelSetpoint = prefs.getDouble("flywheelSetpoint", 0);
      flywheel.getPIDController().setSetpoint(flywheelSetpoint);
    } else if (tuningMode == 2) {
      // Tuning Mode 2 = HSV Filter Testing
      shooter.manualControl(leftStick);
      
      double hMin = prefs.getDouble("hMin", 0);
      double sMin = prefs.getDouble("sMin", 0);
      double vMin = prefs.getDouble("vMin", 0);
      Scalar minRange = new Scalar(hMin, sMin, vMin);
      
      double hMax = prefs.getDouble("hMax", 180);
      double sMax = prefs.getDouble("sMax", 255);
      double vMax = prefs.getDouble("vMax", 255);
      Scalar maxRange = new Scalar(hMax, sMax, vMax);
      
      Mat result = imgProcessor.getHSVFilter(minRange, maxRange);
      camController.pushFrameToDash(result);
    } else if (tuningMode == 3) {
      // Tuning Mode 3 = Contour Filter Testing
      shooter.manualControl(leftStick);
      
      double minArea = prefs.getDouble("minArea", 0);
      double minShapeScore = prefs.getDouble("minShapeScore", 0);
      double maxShapeScore = prefs.getDouble("maxShapeScore", 2);
      
      Scalar minHsvRange = imgProcessor.DEFAULT_HSV_LOWER_BOUNDS;
      Scalar maxHsvRange = imgProcessor.DEFAULT_HSV_UPPER_BOUNDS;
      Mat result = imgProcessor.getHSVFilter(minHsvRange, maxHsvRange);
      
      ArrayList<MatOfPoint> contours = imgProcessor.findContours(result);
      for (int i = contours.size()-1; i >= 0; i--) {
        boolean test1 = imgProcessor.passesAreaTest(contours.get(i), minArea);
        boolean test2 = imgProcessor.passesShapeTest(contours.get(i), minShapeScore, maxShapeScore);
        if (!(test1 && test2)) {
          contours.remove(i);
        }
      }
      
      result = imgProcessor.drawContours(true, contours);
      camController.pushFrameToDash(result);
    }
    
    // Publish all data to smart dash
    shooter.publishDataToSmartDash();
    imgProcessor.publishDataToSmartDash();
  }

  /**
   * This function is called once upon entering "disabled" mode.
   */
  public void disabledInit() {
    shooter.stop();
  }
  
  public void testInit() {
    
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {
    
  }
}