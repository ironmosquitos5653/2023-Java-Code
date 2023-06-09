// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int deviceIDadvance = 24;
  private CANSparkMax advance;
  private static final int deviceIDadvance2 = 23;
  private CANSparkMax advance2;

  private static final int deviceIDOverTheBumper = 30;
  private CANSparkMax overTheBumper;

  private static final int solenoidForward = 2;
  private static final int solenoidReverse = 3;
  private DoubleSolenoid theSolenoid;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    advance = new CANSparkMax(deviceIDadvance, MotorType.kBrushless);
    advance.restoreFactoryDefaults();
    advance2 = new CANSparkMax(deviceIDadvance2, MotorType.kBrushless);
    advance2.restoreFactoryDefaults();
    overTheBumper = new CANSparkMax(deviceIDOverTheBumper, MotorType.kBrushless);
    overTheBumper.restoreFactoryDefaults();
    theSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, solenoidForward, solenoidReverse);
    intakeOut(false);
  }

  @Override
  public void periodic() {

  }

// add deploy stuff here

  public void shoot(double speed) {
    advance.set(-speed);
    advance2.set(-speed);
  }

  public void dontshoot() {
    advance.set(0);
    advance2.set(0);
  }

  public void setOverBumperSpeed(double speed) {
     overTheBumper.set(-speed);
  }

  public void intakeOut(boolean isOut) {
    if (isOut) {
      theSolenoid.set(Value.kForward);
      overTheBumper.set(-1);
    } else {
      theSolenoid.set(Value.kReverse);
      overTheBumper.set(0);
    }
  }

  public void toggleIntake() {
  if (theSolenoid.get() == Value.kForward) {
    overTheBumper.set(0);
  } else {
    overTheBumper.set(-1);
  }
    theSolenoid.toggle();
  }
}
