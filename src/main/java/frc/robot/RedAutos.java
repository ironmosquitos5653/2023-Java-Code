package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RedAutos {
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private TrajectoryCommandFactory m_trajectoryCommandFactory;

    public Command right3BallAuto;
    private Command left3BallAuto;
    private Command centerBalanceCommand;

    // Side Positions
    private final double Y_BALL = 4.9;
    private final double Y_CHARGE = 3.7;
    private final double Y_BALANCE = 3;

    private final double X_OFFSET = -.2032;
    private final double X_3BALLOFFSET = 0.6;
    private final double X_SHOOTOFFSET = .3;

    // Center positions
    private final double Y_MOBILITY = 5;
    private final double Y_CENTERBALL = 6.4;
    private final double X_CENTEROFFSET = -.3;

    public RedAutos(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
        m_driveSubsystem = driveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_trajectoryCommandFactory = trajectoryCommandFactory;

        right3BallAuto = buildSideThreeBall(1);
        left3BallAuto = buildSideThreeBall(-1);
        centerBalanceCommand = buildBalanceCommand();
    }

    public Command buildSideThreeBall(double t) {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(X_OFFSET, t * X_OFFSET)
                ),
            new Pose2d(Y_BALL, t * X_OFFSET, new Rotation2d(0))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory driveIn = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BALL, t * X_OFFSET, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, new Rotation2d(0))
        );
        Command driveInCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn);

        Trajectory driveOut3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, new Rotation2d(0)),
            List.of(
                new Translation2d()
                ),
                new Pose2d(Y_BALL, t * X_3BALLOFFSET, new Rotation2d(-30))
        );
        Command driveOut3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut3Ball);

        Trajectory driveIn3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BALL, t * X_3BALLOFFSET, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, new Rotation2d(t * 15))
        );
        Command driveIn3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn3Ball);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOutCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveInCommand)
        // FishingOn
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        //FishingIn
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOut3BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveIn3BallCommand)
         // FishingOn
         .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
         //FishingIn
        ;
    }

    private Command buildBalanceCommand() {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Y_MOBILITY, 0)
                ),
            new Pose2d(Y_BALANCE, 0, new Rotation2d(0))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        .andThen(driveOutCommand)
        .andThen(new BalanceCommand(m_driveSubsystem))
        ;
    }

    private Command buildBalancePlus1Command(double t) {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d()
                ),
            new Pose2d(Y_MOBILITY, 0, new Rotation2d(0))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory ballPickup = m_trajectoryCommandFactory.createTrajectory(
             new Pose2d(Y_MOBILITY, 0, new Rotation2d(0)),
             List.of(new Translation2d()),
             new Pose2d(Y_CENTERBALL, X_CENTEROFFSET, new Rotation2d(-20))
         );
         Command ballPickupCommand = m_trajectoryCommandFactory.createTrajectoryCommand(ballPickup);

         Trajectory balanceTraj = m_trajectoryCommandFactory.createTrajectory(
             new Pose2d(Y_CENTERBALL, X_CENTEROFFSET, new Rotation2d(0)),
             List.of(new Translation2d(Y_MOBILITY, 0)),
             new Pose2d(Y_BALANCE, 0, new Rotation2d(30))
         );
         Command balanceTrajCommand = m_trajectoryCommandFactory.createTrajectoryCommand(balanceTraj);


        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        .andThen(driveOutCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(ballPickupCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(balanceTrajCommand)
        .andThen(new BalanceCommand(m_driveSubsystem))
        ;
    }
}
