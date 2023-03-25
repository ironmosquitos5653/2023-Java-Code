// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {

  private static final int deviceIDpayout = 31;
  private CANSparkMax payout;

  /** Creates a new ArmSubsystem. */
  public ExtenderSubsystem() {


    payout = new CANSparkMax(deviceIDpayout, MotorType.kBrushless);
    payout.restoreFactoryDefaults();
    payout.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Payout", payout.getEncoder().getPosition());

  }

  public void runPayout(double speed) {
    payout.set(speed);
  }

  public void payoutIn(double speed) {
    payout.set(-speed);
  }

  public void stop() {
    payout.set(0);
  }

  public double getPayoutEncoderPosition() {
    return payout.getEncoder().getPosition();
  }
}
