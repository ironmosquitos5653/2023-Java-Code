// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;

  /** Creates a new BalanceCommand. */
  public BalanceCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_driveSubsystem.getPitchDegrees() > 11) {
      m_driveSubsystem.drive(-.13, 0, 0, isFinished());
    } else if(m_driveSubsystem.getPitchDegrees() < -11) {
      m_driveSubsystem.drive(.13, 0, 0, isFinished());
    } else {
      m_driveSubsystem.drive(0, 0, 0, isFinished());
      m_driveSubsystem.setX();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setX();
  }

  Timer timer = null;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_driveSubsystem.getPitchDegrees() < 2 && m_driveSubsystem.getPitchDegrees() > -2) {
      if(timer == null) {
        timer = new Timer();
        timer.start();
      }
      if(timer.hasElapsed(.25)) {
        return true;
      }
    } else {
      timer = null;
    }
    return false;
  }
}
