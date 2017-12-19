package org.usfirst.frc.team1787.robot.subsystems;

import org.usfirst.frc.team1787.robot.utils.CustomPIDController;

/**
 * This interface is included to act as a guide / checklist for what methods to include
 * in each subsystem's class. Please note that this interface isn't actually 
 * implemented by any of the subsystem classes due to some slight variations in how
 * each are controlled.
 * 
 * Each class can be thought of as a "State Controller" for a particular mechanism. For example,
 * imagine that you have an arm on your robot that can either be extended or retracted. Those 2 configurations are 
 * the different states that the arm can exist in (excluding the "transient" state of transitioning from 1 to the other)
 * In order to control the arm, you want a way to be able to manipulate it's state. This corresponds to setters in your classes.
 * What is the "state" of a mechanism? It's just a set of numbers that describe the configuration of the mechanism 
 * 
 * The methods you should have include:
 * 1) setters for each of the components that have state.
 *  -consider making frequent use cases their own method to improve clarity and maintain clean code!!!
 *  -includes manual control (make controls make sense and feel intutitive!)
 *  -includes sensor resetting
 * 2) getters for each of the components that have state.
 *  -reading data from sensors
 *  -reading data from PIDControllers?
 * 3) a safe stop method
 * 4) a singleton instance getter
 * @author Simon
 *
 * @param <T>
 */
public interface Subsystem<T> {
  
  /**
   * It's useful for each subsystem to have a stop() method
   * that you know can be called in the case of an emergency 
   * or whenever you need to completely halt a mechanism
   */
  public void stop();
  
  /**
   * It's useful for each subsystem to have some type of manualControl method
   * as it provides an easy means to make sure your mechanisms are working correctly 
   * or to fine tune certain default values, like a default speed.
   * @param value
   */
  public void manualControl(double value);
  
  /**
   * It seems to be a good idea to utilize a "Singleton Design Pattern" in the code.
   * What this means is that each subsystem class only has one instance associated with it, 
   * called a singleton. By only allowing one instance of a class, you can be sure that there 
   * aren't two instances of the class trying to control the same mechanism at the same time with
   * conflicting / different methods.
   * 
   * The way to implement a "Singleton Design Pattern" is to make the constructors for your subsystems 
   * private. Then include a single instance of the class within itself, which will be your singleton instance.
   * By making the constructor private, you prevent an instance of your subsystem from being created outside of its class. 
   * Whenever you need to control that subsystem, simply get the object that controls it through the getInstance() method.
   * @return
   */
  public T getInstance();
  
  /*
   * For if there are Sensors:
   */
  
  /**
   * For mechanisims that are controlled by PIDControllers, its very handy to be able to access
   * the controller in different parts of the code.
   * @return
   */
  public CustomPIDController getPIDController();
  
  /**
   * It's quite useful to have a method that will quickly allow you to reset 
   * all of the sensors associated with a subsystem.
   */
  public void zeroSensors();
  
  /**
   * By publishing any sort of data that you might want to see on the SmartDashboard
   * all in a single method, you greatly increase the organization of your code.
   * It also makes the code easier to edit.
   */
  public void publishDataToSmartDash();
}
