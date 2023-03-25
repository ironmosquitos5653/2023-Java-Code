// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int deviceIDadvance = 24;
  private CANSparkMax advance;
  private static final int deviceIDadvance2 = 23;
  private CANSparkMax advance2;

  private DigitalInput inSwitch;
  private DigitalInput outSwitch;
  private Spark spark1;
  private Spark spark2;
  private static final int deviceIDOverTheBumper = 30;
  private CANSparkMax overTheBumper;
  private boolean intakeIsOut = false;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    advance = new CANSparkMax(deviceIDadvance, MotorType.kBrushless);
    advance.restoreFactoryDefaults();
    advance2 = new CANSparkMax(deviceIDadvance2, MotorType.kBrushless);
    advance2.restoreFactoryDefaults();
    overTheBumper = new CANSparkMax(deviceIDOverTheBumper, MotorType.kBrushless);
    overTheBumper.restoreFactoryDefaults();

    inSwitch = new DigitalInput(2);
    addChild("InSwitch", inSwitch);

    outSwitch = new DigitalInput(3);
    addChild("OutSwitch", outSwitch);

    spark1 = new Spark(0);
    addChild("Spark",spark1);
    spark1.setInverted(true);

    spark2 = new Spark(1);
    addChild("Spark",spark2);
    spark2.setInverted(false);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isOut", out());
    SmartDashboard.putBoolean("inIn", in());
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

  private static final double defaultIntakeSpeed = -.7;
  boolean isOut = false;
  double intakeSpeed = .7;

  public void stop() {
    spark1.set(0);
    spark2.set(0);
  }

  public boolean in() {
    return !inSwitch.get();
  }

  public boolean out() {
    return !outSwitch.get();
  }


  public void start() {
    spark1.set(intakeSpeed);
    spark2.set(intakeSpeed);
   }
 
 public void reverse() {
   spark1.set(-intakeSpeed);
   spark2.set(-intakeSpeed);
 } 

  public void setOverBumperSpeed(double speed) {
    overTheBumper.set(-speed);
  }

  public void toggle() {
    intakeIsOut = !intakeIsOut;
    if (intakeIsOut)
      overTheBumper.set(defaultIntakeSpeed);
  }

  public void go() {
    if(in() && !intakeIsOut) {
      overTheBumper.set(0);
    }
    if(!intakeIsOut && !in()) {
      reverse();
    } else if(intakeIsOut && !out()) {
      start();
    } else {
      stop();
    }
  }

  public void setIntakeOut(boolean isOut) {
    intakeIsOut = isOut; 
    if (intakeIsOut)
        overTheBumper.set(defaultIntakeSpeed);
  }
}
