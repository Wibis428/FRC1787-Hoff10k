package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.utils.CustomPIDController;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
  
  // Talon
  private final int TURRET_TALON_ID = 1;
  private CANTalon turretMotor = new CANTalon(TURRET_TALON_ID);

  // Gyro
  private final int TURRET_GYRO_ANALOG_PORT = 0;
  private AnalogGyro gyro = new AnalogGyro(TURRET_GYRO_ANALOG_PORT);

  // PID Controller Gains / Configuration Preferences
  private final double TURRET_PID_CONTROLLER_KP = 0;
  private final double TURRET_PID_CONTROLLER_KI = 0;
  private final double TURRET_PID_CONTROLLER_KD = 0;
  private final double TURRET_PID_ABSOLUTE_TOLERENCE_IN_DEGREES = 0;
  private CustomPIDController turretController = new CustomPIDController(TURRET_PID_CONTROLLER_KP,
                                                             TURRET_PID_CONTROLLER_KI,
                                                             TURRET_PID_CONTROLLER_KD,
                                                             0, gyro, turretMotor, 
                                                             PIDController.kDefaultPeriod);
  
  // Singleton Instance
  private static final Turret instance = new Turret();
  
  private Turret() {
    // Configure PID Controller
    turretController.setAbsoluteTolerance(TURRET_PID_ABSOLUTE_TOLERENCE_IN_DEGREES);
    gyro.calibrate();
  }
  
  public CustomPIDController getPIDController() {
    return turretController;
  }
  
  public AnalogGyro getGyro() {
    return gyro;
  }
  
  public void zeroSensors() {
    gyro.reset();
  }

  public void manualControl(double value) {
    if (turretController.isEnabled()) {
      turretController.reset();
    }
    turretMotor.set(value);
  }
  
  public void stop() {
    manualControl(0);
  }

  public void publishDataToSmartDash() {
    SmartDashboard.putBoolean("Turret PID Enabled", turretController.isEnabled());
    SmartDashboard.putNumber("turretAngle", gyro.getAngle());
    SmartDashboard.putNumber("turretError", turretController.getError());
    SmartDashboard.putNumber("turretMotorOutput", turretController.get());
    SmartDashboard.putBoolean("Turret On Target", turretController.onTarget());
  }
  
  public static Turret getInstance() {
    return instance;
  }
}
