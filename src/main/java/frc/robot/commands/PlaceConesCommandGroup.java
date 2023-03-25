// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConesCommandGroup extends ParallelCommandGroup {
  /** Creates a new PlaceConesCommandGroup. */
  public PlaceConesCommandGroup(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem, double position, double extendPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmHoldCommand(armSubsystem, position),
      new MovePayoutCommand(extenderSubsystem, extendPosition)
    );
    
  }
}
