// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  public ShooterSubsystem m_shooterSubsystem;
  public IntakeSubsystem m_intakeSubsystem;
  public double m_speed;
  Timer timer = new Timer();

  public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_speed = speed;
    addRequirements(m_shooterSubsystem);
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.shoot(m_speed);
    if(timer.hasElapsed(.3)) {
      m_shooterSubsystem.shootadvance(m_speed);
      m_intakeSubsystem.shoot(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.dontshoot();
    m_intakeSubsystem.dontshoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
