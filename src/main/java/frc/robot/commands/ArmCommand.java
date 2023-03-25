// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private ArmSubsystem m_armSubsystem;
  private double m_position;

  public ArmCommand(ArmSubsystem armSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    m_position = position;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_armSubsystem.setRotateSpeed(-.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stop();
    System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DONE!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.getEncoderPosition() < m_position;
  }
}
