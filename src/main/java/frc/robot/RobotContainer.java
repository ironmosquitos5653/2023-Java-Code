// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmDeployCommandGroup;
import frc.robot.commands.ArmHoldCommand;
import frc.robot.commands.AutonomousOneCommandGroup;
import frc.robot.commands.BalanceAutoCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.MovePayoutCommand;
import frc.robot.commands.NoAutoCommand;
import frc.robot.commands.PayoutInCommand;
import frc.robot.commands.PlaceConesCommandGroup;
import frc.robot.commands.ResetArmCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SpitConeCommand;
import frc.robot.commands.ToggleColorCommand;
import frc.robot.commands.ToggleConeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.blinkin;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Constants.PWMPorts;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  public final blinkin m_blinkinSubsystem = new blinkin(3);
  public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
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


      joystick.button(4).whileTrue(new ResetArmCommand(m_ArmSubsystem, .3));
      joystick.button(3).whileTrue(new ResetArmCommand(m_ArmSubsystem, -.3));

      joystick.button(1).whileTrue(new ToggleConeCommand(m_ArmSubsystem));
      joystick.button(2).whileTrue(new SpitConeCommand(m_ArmSubsystem));

      joystick.button(6).whileTrue(new MovePayoutCommand(m_ExtenderSubsystem, 0));
      joystick.button(5).whileTrue(new PayoutInCommand(m_ExtenderSubsystem, 0));

      m_driverController.rightBumper().whileTrue(new DeployIntakeCommand(m_intakeSubsystem));
      m_driverController.leftBumper().whileTrue(new BrakeCommand(m_robotDrive));

      // Score Low
      m_driverController.a().onTrue(new ArmDeployCommandGroup(m_ArmSubsystem, m_ExtenderSubsystem, -35, 0));
      // Score High
      m_driverController.b().onTrue(new ArmDeployCommandGroup(m_ArmSubsystem, m_ExtenderSubsystem, -35, 58));

    //m_driverController.y().onTrue(new ToggleColorCommand(m_blinkinSubsystem));

    //   abutton.whenPressed(() -> m_blinkin.set(.91), m_blinkin);    // purple
    //   bbutton.whenPressed(() -> m_blinkin.set(.69), m_blinkin);   // yellow

    SmartDashboard.putData(new ArmHoldCommand(m_ArmSubsystem, -35));
    SmartDashboard.putData(new MovePayoutCommand(m_ExtenderSubsystem, 71));
    SmartDashboard.putData(new ArmDeployCommandGroup(m_ArmSubsystem, m_ExtenderSubsystem, -45, 20));
    SmartDashboard.putData(new PlaceConesCommandGroup(m_ArmSubsystem, m_ExtenderSubsystem, -45, 20));
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

    
