// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private static final int deviceIDcone = 32;
  private CANSparkMax coneIntake;
  private static final int deviceIDrotate = 33;
  private CANSparkMax rotate;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    rotate = new CANSparkMax(deviceIDrotate, MotorType.kBrushless);
    rotate.restoreFactoryDefaults();
    rotate.getEncoder().setPosition(0);
    coneIntake = new CANSparkMax(deviceIDcone, MotorType.kBrushless);
    coneIntake.restoreFactoryDefaults();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotate", rotate.getEncoder().getPosition());

  }

  public void stop() {
    rotate.set(0);
  }

  public void setRotateSpeed(double speed) {
    rotate.set(speed);
  }

  public double getEncoderPosition() {
    return rotate.getEncoder().getPosition();
  }

  public void setIntakeSpeed(double speed) {
    coneIntake.set(speed);
  }

  public void spitIntakeSpeed(double speed) {
    coneIntake.set(-speed);
  }

  public void toggleIntakeMotor() {
    if(coneIntake.get() == 0) {
      coneIntake.set(1);
    } else {
      coneIntake.set(0);
    }
    SmartDashboard.putNumber("intake motor", coneIntake.get());
  }

}
