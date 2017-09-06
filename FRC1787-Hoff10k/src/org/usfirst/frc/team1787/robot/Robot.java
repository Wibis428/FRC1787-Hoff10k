package org.usfirst.frc.team1787.robot;

import org.usfirst.frc.team1787.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1787.robot.subsystems.Flywheel;
import org.usfirst.frc.team1787.robot.subsystems.PickupArm;
import org.usfirst.frc.team1787.robot.subsystems.Shooter;
import org.usfirst.frc.team1787.robot.subsystems.Turret;
import org.usfirst.frc.team1787.robot.subsystems.Winch;
import org.usfirst.frc.team1787.robot.utils.CustomJoystick;
import org.usfirst.frc.team1787.robot.vision.CameraController;
import org.usfirst.frc.team1787.robot.vision.Target;
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
  private final int WINCH_DESCEND_BUTTON = 7;
  /* The ratchet on the winch motor prevents the winch from rotating in the opposite direction,
   * so we don't have to check if the WINCH_DESCEND_BUTTON is being pressed. However, it's value remains
   * here for if we ever decide to remove the ratchet. */
  
  private final int TOGGLE_SHOOTER_CONTROL_BUTTON = 2;
  private final int TOGGLE_CAM_BUTTON = 10;
  
  private final int TOGGLE_TUNING_MODE_BUTTON = -1;
  private final int CYCLE_THROUGH_TUNING_MODES_BUTTON = -1;
  private boolean tuningModeActive = false;
  private int tuningMode = 0;
  
  // Instances of Subsystems
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private PickupArm pickupArm = PickupArm.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Winch winch = Winch.getInstance();
  private CameraController camController = CameraController.getInstance();
  
  // These are only used for tuning
  private Flywheel flywheel = Flywheel.getInstance();
  private Turret turret = Turret.getInstance();
  
  // Preferences
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
      driveTrain.regularArcadeDrive(rightStick);
    } else {
      /* Use of the left stick is currently reserved for the shooter. */
      //driveTrain.reverseArcadeDrive(leftStick);
    }
    
    // Gear Shifter
    if (rightStick.getSlider() < 0) {
      driveTrain.setHighGear();
    } else {
      driveTrain.setLowGear();
    }
    driveTrain.publishDataToSmartDash();
    
    // Pickup Arm
    if (rightStick.getRawButton(DEPLOY_ARM_BUTTON)) {
      pickupArm.deploy();
    } else if (rightStick.getRawButton(RETRACT_ARM_BUTTON)) {
      pickupArm.retract();
    }
    
    // Pickup Wheels
    if (rightStick.getRawButton(INTAKE_BUTTON)) {
      pickupArm.intake();
    } else if (rightStick.getRawButton(EXPELL_BUTTON)) {
      pickupArm.expell();
    } else {
      pickupArm.stopPickupWheels();
    }
    
    // Winch
    if (rightStick.getRawButton(WINCH_CLIMB_BUTTON)) {
      winch.climb();
    } else {
      winch.stop();
    }
    
    // Tuning Mode
    if (leftStick.getSinglePress(TOGGLE_TUNING_MODE_BUTTON)) {
      tuningModeActive = !tuningModeActive;
      SmartDashboard.putBoolean("Tuning Mode Active", tuningModeActive);
    }
    
    if (!tuningModeActive) {
      // Shooter
      if (leftStick.getSinglePress(TOGGLE_SHOOTER_CONTROL_BUTTON)) {
        if (shooter.pidIsEnabled()) {
          shooter.stop(); // disables PID Controllers
        } else {
          shooter.enablePIDControllers();
        }
      }
      
      if (shooter.pidIsEnabled()) {
        shooter.fullAutoShooting();
      } else {
        shooter.manualControl(leftStick);
      }

      // Cams (no processing)
      if (rightStick.getSinglePress(TOGGLE_CAM_BUTTON)) {
        camController.toggleCamStream();
      }
    } else {
      runTuningCode();
    }
    shooter.publishDataToSmartDash();
    camController.publishDataToSmartDash();
  }
  
  public void runTuningCode() {
    if (leftStick.getSinglePress(CYCLE_THROUGH_TUNING_MODES_BUTTON)) {
      tuningMode = (tuningMode + 1) % 4;
    }
    
    if (tuningMode == 0) {
      // Tuning Mode 0 = Turret PID Testing
      if (!shooter.pidIsEnabled()) {
        shooter.stop();
        double turretP = prefs.getDouble("turretP", 0);
        double turretI = prefs.getDouble("turretI", 0);
        double turretD = prefs.getDouble("turretD", 0);
        turret.getPIDController().setPID(turretP, turretI, turretD);
        double turretTolerance = prefs.getDouble("turretDegreesTolerance", 0);
        turret.getPIDController().setAbsoluteTolerance(turretTolerance); 
        double testFocalLength = prefs.getDouble("testFocalLength", 146.9);
        Target.setFocalLength(testFocalLength);
        turret.getPIDController().enable();
      }
      shooter.trackTarget();
    } else if (tuningMode == 1) {
      // Tuning Mode 1 = Flywheel PID Testing
      if (!shooter.pidIsEnabled()) {
        shooter.stop();
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
      double hMax = prefs.getDouble("hMax", 180);
      double sMax = prefs.getDouble("sMax", 255);
      double vMax = prefs.getDouble("vMax", 255);
      camController.setHSVBounds(hMin, sMin, vMin, hMax, sMax, vMax);
      camController.showHSVFilter();
    } else if (tuningMode == 3) {
      // Tuning Mode 3 = Contour Filter Testing
      shooter.manualControl(leftStick);
      double minArea = prefs.getDouble("minArea", 0);
      double minShapeScore = prefs.getDouble("minShapeScore", 0);
      double maxShapeScore = prefs.getDouble("maxShapeScore", 2);
      Target.setMinArea(minArea);
      Target.setShapeScoreBounds(minShapeScore, maxShapeScore);
      camController.showContoursfilter(true);
    }
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