// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

public class ArmHoldOffCommand extends CommandBase {
  /** Creates a new ArmHoldOffCommand. */

  public ArmSubsystem m_armSubsystem;
  public ExtenderSubsystem m_extenderSubsystem;
  private double dropTime = .5;

  public ArmHoldOffCommand(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  Timer timer = new Timer();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    ArmHoldCommand.endCommand = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(dropTime)) {
      m_armSubsystem.setIntakeSpeed(-.5);
    } else {
      m_armSubsystem.setIntakeSpeed(0);
    }
    if(timer.hasElapsed(dropTime) && m_extenderSubsystem.getPayoutEncoderPosition() > 0) {
      m_extenderSubsystem.runPayout(-.5);
    } else {
      m_extenderSubsystem.runPayout(0);
    }
    if(timer.hasElapsed(dropTime) && m_extenderSubsystem.getPayoutEncoderPosition() <= 0 && m_armSubsystem.getEncoderPosition() > -20) {
      m_armSubsystem.setRotateSpeed(.3);
    } else {
      m_armSubsystem.setRotateSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stop();
    m_extenderSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.getEncoderPosition() > -20;
  }
}
