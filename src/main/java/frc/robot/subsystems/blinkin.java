// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class blinkin extends SubsystemBase {

  private static Spark m_blinkin = null;
  /** Creates a new blinkin. */
  public blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
  }
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }
  public void solid_yellow() {
    set(0.69);
  }
  boolean isYellow = false;
  public void toggleColor() {
    if (isYellow = true){
      m_blinkin.set(0.91);
    } else {
      m_blinkin.set(0.69);
    }
  }
}