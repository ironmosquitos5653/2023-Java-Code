// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  //private static RobotContainer m_robotContainer = new RobotContainer();
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final ExtenderSubsystem m_ExtenderSubsystem = new ExtenderSubsystem();

  // The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(0); // owenwashere
  public static CommandJoystick joystick = new CommandJoystick(1);
  //public static blinkin m_blinkin = new blinkin(PWMPorts.kBlinkin);
  public final AutonomousManager m_autonomousManager;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  

  public RobotContainer() {
    m_autonomousManager = new AutonomousManager(m_robotDrive, m_intakeSubsystem, m_shooterSubsystem);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
               true),
            m_robotDrive));
    m_robotDrive.setX();

  }


  //public static RobotContainer getInstance() {
   // return m_robotContainer;
  
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {  
    

      //joystick.button(2).onTrue(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1));
      joystick.button(8).onTrue(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .4));
      joystick.button(10).onTrue(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .6));
      joystick.button(12).onTrue(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .9));

      m_driverController.leftBumper().whileTrue(new BrakeCommand(m_robotDrive));


    //m_driverController.y().onTrue(new ToggleColorCommand(m_blinkinSubsystem));

    //   abutton.whenPressed(() -> m_blinkin.set(.91), m_blinkin);    // purple
    //   bbutton.whenPressed(() -> m_blinkin.set(.69), m_blinkin);   // yellow

    SmartDashboard.putData(new BalanceCommand(m_robotDrive));

  }

  // public Command AutonomousCommand() {
  //   return m_chooser.getSelected();
  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. 
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return m_autonomousManager.getAutonomousCommand();
  }
}

    
