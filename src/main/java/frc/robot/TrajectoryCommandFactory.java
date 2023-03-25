package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryCommandFactory {

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    private final TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    private DriveSubsystem m_robotDrive;

    public TrajectoryCommandFactory(DriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Trajectory createTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }


    public SwerveControllerCommand createTrajectoryCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, .1, 0),
            new PIDController(AutoConstants.kPYController, .1, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }
    
}