package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutonomousOneCommandGroup;
import frc.robot.commands.BalanceAutoCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.NoAutoCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousManager {
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private TrajectoryCommandFactory m_trajectoryCommandFactory;

    private Command rightPipeDreamCommand;
    private Command rightNoBalanceCommand;
    private Command rightBalanceCommand;
    private Command leftPipeDreamCommand;
    private Command centerRightBalanceCommand;

    public SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    public AutonomousManager(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_trajectoryCommandFactory = new TrajectoryCommandFactory(driveSubsystem);
        initRightCommand();
        initCenterCommand();
        m_chooser.addOption("CenterRight", centerRightBalanceCommand);
        m_chooser.addOption("AutonomousCommand", new AutonomousOneCommandGroup(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem));
        m_chooser.addOption("BalanceAutonomousCommand", new BalanceAutoCommandGroup(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
        m_chooser.addOption("RightNoBalance", rightNoBalanceCommand);
        m_chooser.addOption("RightBalanceCommand", rightBalanceCommand);
        //m_chooser.addOption("RightPipeDream", rightPipeDreamCommand);
        m_chooser.addOption("No Auto", new NoAutoCommand());
        SmartDashboard.putData("Auto Mode", m_chooser);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    private void initRightCommand() {
       Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(.2032, -.2032), new Translation2d(4.7, -.2032)),
            new Pose2d(3.7, -.2032, new Rotation2d(0))
        );
        SwerveControllerCommand driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);
        SwerveControllerCommand driveOutCommand2  = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory secondCube = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(3.7, -.2032, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(4.9, .6, new Rotation2d(-30))
        );
        SwerveControllerCommand secondCubeCommand = m_trajectoryCommandFactory.createTrajectoryCommand(secondCube);
        SwerveControllerCommand secondCubeCommand2 = m_trajectoryCommandFactory.createTrajectoryCommand(secondCube);

        Trajectory upTop = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(4.9, .6, new Rotation2d(0)),
            List.of(new Translation2d(3.5, 1.7526)),
            new Pose2d(4.2, 1.5, new Rotation2d(0))
        );
        SwerveControllerCommand upTopCommand = m_trajectoryCommandFactory.createTrajectoryCommand(upTop);
        SwerveControllerCommand upTopCommand2 = m_trajectoryCommandFactory.createTrajectoryCommand(upTop);



        rightNoBalanceCommand = new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        //.andThen(Commands.runOnce(() -> m_driveSubsystem.resetOdometry(driveOut.getInitialPose())))
        .andThen(
            (driveOutCommand
            .alongWith(new AutoIntakeCommand(m_intakeSubsystem, false))
            )
        )
        //.andThen(Commands.runOnce(() -> m_driveSubsystem.resetOdometry(driveIn.getInitialPose())))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .9))
        .andThen(secondCubeCommand)
        .andThen(upTopCommand)
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        ;

        rightBalanceCommand = new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        //.andThen(Commands.runOnce(() -> m_driveSubsystem.resetOdometry(driveOut.getInitialPose())))
        .andThen(
            (driveOutCommand2
            .alongWith(new AutoIntakeCommand(m_intakeSubsystem, false))
            )
        )
        //.andThen(Commands.runOnce(() -> m_driveSubsystem.resetOdometry(driveIn.getInitialPose())))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .9))
        .andThen(secondCubeCommand2)
        .andThen(upTopCommand2)
        .andThen(
         new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1)
        .alongWith(new BalanceCommand(m_driveSubsystem))
        )
        ;
/*
        rightPipeDreamCommand = rightNoBalanceCommand
        .andThen(Commands.runOnce(() -> m_driveSubsystem.resetOdometry(driveIn.getInitialPose())))
        .andThen(secondCubeCommand)
        .andThen(
            (upTopCommand
            .alongWith(new AutoIntakeCommand(m_intakeSubsystem, true))
            )
        )
        .andThen(new BalanceCommand(m_driveSubsystem))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .9))
        ;*/
    }

    private void initCenterCommand() {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
             new Pose2d(0, 0, new Rotation2d(0)),
             new ArrayList<Translation2d>(),
             new Pose2d(5, 0, new Rotation2d(0))
         );
         SwerveControllerCommand driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

         Trajectory driveOut2 = m_trajectoryCommandFactory.createTrajectory(
             new Pose2d(5, 0, new Rotation2d(0)),
             new ArrayList<Translation2d>(),
             new Pose2d(6.4, -.3, new Rotation2d(-20))
         );
         SwerveControllerCommand driveOutCommand2 = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut2);

         Trajectory drivein = m_trajectoryCommandFactory.createTrajectory(
             new Pose2d(6.4, -.3, new Rotation2d(0)),
             List.of(new Translation2d(4, 0)),
             new Pose2d(3, 0, new Rotation2d(0))
         );
         SwerveControllerCommand driveInCommand = m_trajectoryCommandFactory.createTrajectoryCommand(drivein);

         centerRightBalanceCommand = 
            new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
            .andThen(driveOutCommand)
            .andThen(new AutoIntakeCommand(m_intakeSubsystem, false))
            .andThen(driveOutCommand2)
            .andThen(new AutoIntakeCommand(m_intakeSubsystem, true))
            .andThen(driveInCommand)
            .andThen(new BalanceCommand(m_driveSubsystem))
            .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1));
    }
}