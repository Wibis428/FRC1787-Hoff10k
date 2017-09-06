package org.usfirst.frc.team1787.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMethods {

  private SendableChooser<Integer> autoChooser;

  private int selectedAuto;

  public AutoMethods() {
    autoChooser = new SendableChooser<Integer>();
    addOptionsToChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void addOptionsToChooser() {
    autoChooser.addDefault("auto1", 1);
    autoChooser.addObject("auto2", 2);
    autoChooser.addObject("auto3", 3);
  }

  public void getSelectedAutoRoutine() {
    selectedAuto = autoChooser.getSelected();
  }

  public void runSelectedAutoRoutine() {
    if (selectedAuto == 1) {
      auto1();
    }
  }

  public void auto1() {

  }
}
