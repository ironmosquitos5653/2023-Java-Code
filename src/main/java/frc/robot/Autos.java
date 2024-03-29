package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Autos {
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private TrajectoryCommandFactory m_trajectoryCommandFactory;

    public Command right3BallAuto;
    public Command left3BallAuto;
    public Command centerBalanceAuto;
    public Command centerRight2BallAuto;
    public Command centerLeft2BallAuto;
    public Command left4BallAuto;
    public Command left5BallAuto;
    public Command right4BallAuto;
    public Command right5BallAuto;
    public Command rightSide3BalanceAuto;
    public Command leftSide3BalanceAuto;
    public Command rightSide2BalanceAuto;
    public Command leftSide2BalanceAuto;

    // Side Positions
    private final double Y_BALL = 5.0;
    private final double Y_CHARGE = 3.7;
    private final double Y_BALANCE = 3.3;

    private final double X_OFFSET = -.2032;
    private final double X_3BALLOFFSET = 1.2;
    private final double X_SHOOTOFFSET = 1.0;

    // Center positions
    private final double Y_MOBILITY = 6;
    private final double Y_CENTERBALL = 6.4;
    private final double X_CENTEROFFSET = -.3;

    // Big Autos
    private final double Y_BIG = 2.3;
    private final double X_OFFSET1 = -.11;
    private final double X_OFFSET2 = .9;
    private final double X_OFFSET3 = 2.4;
    private final double X_OFFSET4 = 2.11;

    private final double Y_DROP = 1;
    private final double X_DROP1 = .8;
    private final double X_DROP2 = 2.0;
    private final double X_DROP3 = 2.6924;
    private final double X_DROP4 = 2.6924;

    public Autos(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
        m_driveSubsystem = driveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_trajectoryCommandFactory = trajectoryCommandFactory;

        right3BallAuto = buildSideThreeBall(1);
        left3BallAuto = buildSideThreeBall(-1);
        centerBalanceAuto = buildBalanceCommand();
        centerRight2BallAuto = buildCenter2BallCommand(-1);
        centerLeft2BallAuto = buildCenter2BallCommand(1);
        
        rightSide3BalanceAuto = buildSideBalance3Command(1);
        leftSide3BalanceAuto = buildSideBalance3Command(-1);
        rightSide2BalanceAuto = buildSideBalance2Command(1);
        leftSide2BalanceAuto = buildSideBalance2Command(-1);
    }

    public Command buildSideThreeBall(double t) {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(-X_OFFSET, t * X_OFFSET)
                ),
            new Pose2d(Y_BALL, t * X_OFFSET, new Rotation2d(0))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory driveIn = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BALL, t * X_OFFSET, new Rotation2d(0)),
            new ArrayList<Translation2d>(),

            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, Rotation2d.fromDegrees(3))
        );
        SwerveControllerCommand driveInCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn);

        Trajectory driveOut3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, Rotation2d.fromDegrees(3)),
            new ArrayList<Translation2d>(),
            new Pose2d(Y_BALL, t * X_3BALLOFFSET, new Rotation2d(0))
        );
        Command driveOut3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut3Ball);

        Trajectory driveIn3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BALL, t * X_3BALLOFFSET, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, new Rotation2d(0))
        );
        Command driveIn3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn3Ball);

        Trajectory backup = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_CHARGE, t * X_SHOOTOFFSET, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(Y_CHARGE + .3, t * X_SHOOTOFFSET, new Rotation2d(0))
        );
        Command backupCommand = m_trajectoryCommandFactory.createTrajectoryCommand(backup);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true), m_intakeSubsystem))
        .andThen(driveOutCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false), m_intakeSubsystem))
        .andThen(driveInCommand)
        .andThen(new WaitCommand(.25))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true), m_intakeSubsystem))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false), m_intakeSubsystem))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true), m_intakeSubsystem))
        .andThen(new WaitCommand(.25))
        .andThen(driveOut3BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false), m_intakeSubsystem))
        .andThen(new WaitCommand(.25))
        .andThen(driveIn3BallCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true), m_intakeSubsystem))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false), m_intakeSubsystem))
        .andThen(backupCommand)
        ;
    }

    private Command buildBalanceCommand() {

        Trajectory driveOutX = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(2.8, 0, new Rotation2d(0))
        );
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Y_MOBILITY, 0)
                ),
            new Pose2d(2.8, 0, new Rotation2d(0))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOutX);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .5)
        .andThen(driveOutCommand)
        .andThen(new BalanceCommand(m_driveSubsystem))
        .andThen(Commands.runOnce(() -> m_driveSubsystem.setX()))
        ;
    }

    private Command buildCenter2BallCommand(double t) {
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
        .andThen(Commands.runOnce(() -> m_driveSubsystem.setX()))
        ;
    }

    public Command buildSideFourBall(double t) {
        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d()
                ),
            new Pose2d(Y_BIG, t * X_OFFSET1, new Rotation2d(20))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory driveIn = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET1, new Rotation2d(20)),
            List.of(new Translation2d()),
            new Pose2d(Y_DROP, t * X_DROP1, new Rotation2d(0))
        );
        Command driveInCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn);

        Trajectory driveOut3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_DROP, t * X_DROP1, new Rotation2d(0)),
            List.of(
                new Translation2d()
                ),
                new Pose2d(Y_BIG, t * X_OFFSET2, new Rotation2d(-30))
        );
        Command driveOut3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut3Ball);

        Trajectory driveIn3Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET2, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_DROP, t * X_DROP2, new Rotation2d(t * 15))
        );
        Command driveIn3BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn3Ball);

        Trajectory driveOut4Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_DROP, t * X_DROP2, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_BIG, t * X_OFFSET3, new Rotation2d(-30))
        );
        Command driveOut4BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut4Ball);

        Trajectory driveIn4Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET3, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_DROP, t * X_DROP3, new Rotation2d(t * 15))
        );
        Command driveIn4BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn4Ball);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .7)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOutCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveInCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true)))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false)))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOut3BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveIn3BallCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true)))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false)))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOut3BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveIn3BallCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true)))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false)))
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOut4BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveIn4BallCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true)))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false)))
        ;
    }

    public Command buildSideFiveBall(double t) {
        Trajectory driveOut5Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_DROP, t * X_DROP3, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_BIG, t * X_OFFSET4, new Rotation2d(-30))
        );
        Command driveOut5BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut5Ball);

        Trajectory driveIn5Ball = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET4, new Rotation2d(0)),
            List.of(new Translation2d()),
            new Pose2d(Y_DROP, t * X_DROP4, new Rotation2d(t * 15))
        );
        Command driveIn5BallCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn5Ball);


        return buildSideFourBall(t)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
        .andThen(driveOut5BallCommand)
        .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false)))
        .andThen(driveIn5BallCommand)
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true)))
        .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, 1))
        .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false)))
        ;
    }

    
    public Command buildSideBalance3Command(double t) {

        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(.5,0)
                ),
            new Pose2d(Y_BIG, t * X_OFFSET2, Rotation2d.fromDegrees(t * 50))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory driveIn = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET2, Rotation2d.fromDegrees(t * 50)),
            new ArrayList<Translation2d>(),
            new Pose2d(Y_DROP, t * X_DROP1, Rotation2d.fromDegrees(t * -12))
        );
        Command driveInCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn);

        Trajectory driveOut2 = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_DROP, t * X_DROP1, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(Y_BIG, t * X_OFFSET3, Rotation2d.fromDegrees(t * 60))
        );
        Command driveOut2Command = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut2);

        Trajectory driveIn2 = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET3, Rotation2d.fromDegrees(t * 60)),
            new ArrayList<Translation2d>(),
            new Pose2d(.2, t * X_DROP2, Rotation2d.fromDegrees(t * 5))
        );
        Command driveIn2Command = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn2);

        return new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .6)
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
            .andThen(driveOutCommand)
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false), m_intakeSubsystem))
            .andThen(driveInCommand)

           .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(true), m_intakeSubsystem))
           .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .7))
           .andThen(Commands.runOnce(() -> m_shooterSubsystem.poleOut(false), m_intakeSubsystem))

           .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true), m_intakeSubsystem))
            .andThen(driveOut2Command)
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false), m_intakeSubsystem))

           .andThen(driveIn2Command)
           .andThen(new BalanceCommand(m_driveSubsystem))
           .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .7))
        ;
    }

    
    public Command buildSideBalance2Command(double t) {
        Trajectory miniOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(.2, 0, Rotation2d.fromDegrees(t * -25))
        );

        Command miniOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(miniOut);

        Trajectory driveOut = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(.3, 0, Rotation2d.fromDegrees(t * -25)),
            List.of(
                new Translation2d(.5,0)
                ),
            new Pose2d(Y_BIG, t * X_OFFSET2, Rotation2d.fromDegrees(t * 50))
        );
        Command driveOutCommand = m_trajectoryCommandFactory.createTrajectoryCommand(driveOut);

        Trajectory driveIn2 = m_trajectoryCommandFactory.createTrajectory(
            new Pose2d(Y_BIG, t * X_OFFSET2, Rotation2d.fromDegrees(t * 50)),
            new ArrayList<Translation2d>(),
            new Pose2d(.2, t * X_DROP2, Rotation2d.fromDegrees(t * 5))
        );
        Command driveIn2Command = m_trajectoryCommandFactory.createTrajectoryCommand(driveIn2);

        return miniOutCommand
            .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .6))
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(true)))
            .andThen(driveOutCommand)
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.intakeOut(false), m_intakeSubsystem))

           .andThen(driveIn2Command)
           .andThen(new BalanceCommand(m_driveSubsystem))
           .andThen(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, .7))
        ;
    }

}
