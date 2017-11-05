package org.usfirst.frc.team1787.robot.utils;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * This class is provided to fix some issues with the given PIDController class.
 * Namely, the onTarget() method isn't working due to the way it's implemented,
 * so a fix is provided here.
 * 
 * The problem is that the onTarget method checks to see if the average error is within
 * acceptable bounds. Because the average error is reset every time setSetpoint is called, 
 * and because setSetpoint is being called continuously, there is never a valid average error.
 * Therefore, the included onTaget() method will never return true if setSetpoint() is called
 * continuously.
 * 
 * The fix: Override the onTarget() method so that it just checks the current error rather than
 * the average.
 * 
 * Also adds some additional functionality, which is in the documentation for the methods below.
 */
public class CustomPIDController extends PIDController {
  
  private double absoluteTolerance;

  public CustomPIDController(double p, double i, double d, double f, PIDSource source, PIDOutput output, double period) {
    super(p, i, d, f, source, output, period);
  }
  
  /**
   * returns a boolean indicating if the current error is
   * within acceptable bounds.
   */
  @Override
  public synchronized boolean onTarget() {
    return Math.abs(this.getError()) <= absoluteTolerance;
  }
  
  /**
   * Sets the the acceptable range of errors to 
   * [-tolerance, tolerance]
   */
  @Override
  public synchronized void setAbsoluteTolerance(double tolerance) {
    absoluteTolerance = Math.abs(tolerance);
  }
  
  /**
   * An overrided version of enable which ensures the error
   * will be 0 upon enabling the controller. This way, the controller
   * doesn't immediately start to act on some outdated error when enabled.
   */
  @Override
  public synchronized void enable() {
    if (m_pidInput.getPIDSourceType() == PIDSourceType.kDisplacement) {
      super.setSetpoint(m_pidInput.pidGet());
    } else if (m_pidInput.getPIDSourceType() == PIDSourceType.kRate) {
      super.setSetpoint(0);
    }
    super.enable();
  }
  
  /**
   * sets the setpoint relative to the current state of the mechanism.
   * 
   * For example, say you need the turret to turn 30 degrees to the right and that the current
   * reading on the gyro is 100 degrees. Simply calling setSetpoint(30) would cause the turret 
   * to turn left 70 degrees until the gyro read 30 degrees. However, calling setRelativeSetpint(30)
   * will result in the proper change. This is because this method adds the desired change to the current state.
   * @param setpoint
   */
  public synchronized void setRelativeSetpoint(double setpoint) {
    super.setSetpoint(m_pidInput.pidGet() + setpoint);
  }
}
