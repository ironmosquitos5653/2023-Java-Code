// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int deviceIDshootadv = 26;
  private CANSparkMax shootadvance;
  private static final int deviceIDshootadv2 = 27;
  private CANSparkMax shootadvance2;
  private static final int deviceIDshoot = 25;
  private CANSparkMax shooter;
  private static final int leftServoID = 2;
  private static final int rightServoID = 3;
  private Servo leftServo;
  private Servo rightServo;
  private static final int solenoidForward = 0;
  private static final int solenoidReverse = 1;
  private DoubleSolenoid poleSolenoid;

  /** Creates a new ShooterSubsystem. */
  
  public ShooterSubsystem() {

    shootadvance = new CANSparkMax(deviceIDshootadv, MotorType.kBrushless);
    shootadvance.restoreFactoryDefaults();
    shootadvance2 = new CANSparkMax(deviceIDshootadv2, MotorType.kBrushless);
    shootadvance2.restoreFactoryDefaults();
    shooter = new CANSparkMax(deviceIDshoot, MotorType.kBrushless);
    shooter.restoreFactoryDefaults();
    leftServo = new Servo(leftServoID);
    rightServo = new Servo(rightServoID);
    poleSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, solenoidForward, solenoidReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shoot(double speed) {
    shooter.set(-speed);
  }

  public void shootadvance(double speed) {
    shootadvance2.set(speed);
    shootadvance.set(speed);
  }

  public void dontshoot() {
    shootadvance.set(0);
    shootadvance2.set(0);
    shooter.set(0);
  }

  public void servoOut(boolean isOut) {
    if(isOut) {
      leftServo.setAngle(0);
      rightServo.setAngle(0);
    } else {
      leftServo.setAngle(0);
      rightServo.setAngle(0);
    }
  }

  public void togglePoles() {
    poleSolenoid.toggle();
  }
}
