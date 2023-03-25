// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

public class PayoutInCommand extends CommandBase {
  /** Creates a new MovePayoutCommand. */
  private ExtenderSubsystem m_extenderSubsystem;
  private double m_extenderPosition;

  public PayoutInCommand(ExtenderSubsystem extenderSubsystem, double extenderPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extenderSubsystem = extenderSubsystem;
    addRequirements(m_extenderSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extenderSubsystem.payoutIn(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extenderSubsystem.runPayout(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
