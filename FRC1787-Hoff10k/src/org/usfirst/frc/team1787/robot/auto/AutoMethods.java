package org.usfirst.frc.team1787.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMethods {

  private SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();
  private int selectedAuto;

  public AutoMethods() {
    // Add options to chooser
    autoChooser.addDefault("auto1", 1);
    autoChooser.addObject("auto2", 2);
    autoChooser.addObject("auto3", 3);
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void getSelectedAutoRoutine() {
    selectedAuto = autoChooser.getSelected();
  }

  public void runSelectedAutoRoutine() {
    if (selectedAuto == 1) {
      auto1();
    } else if (selectedAuto == 2) {
      auto2();
    } else if (selectedAuto == 3) {
      auto3();
    }
  }

  public void auto1() {

  }
  
  public void auto2() {
    
  }
  
  public void auto3() {
    
  }
}
