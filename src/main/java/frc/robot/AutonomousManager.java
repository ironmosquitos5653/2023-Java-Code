package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousManager {
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private TrajectoryCommandFactory m_trajectoryCommandFactory;

    private Autos m_autos;

    public SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    public AutonomousManager(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_trajectoryCommandFactory = new TrajectoryCommandFactory(driveSubsystem);
        m_autos = new Autos(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_trajectoryCommandFactory);
        
        m_chooser.addOption("Left 3 Ball", m_autos.left3BallAuto);
        m_chooser.addOption("Right 3 Ball", m_autos.right3BallAuto);
        m_chooser.addOption("Left 4 Ball", m_autos.left4BallAuto);
        m_chooser.addOption("Right 4 Ball", m_autos.right4BallAuto);
        // m_chooser.addOption("Left 5 Ball", m_autos.left5BallAuto);
        // m_chooser.addOption("Right 5 Ball", m_autos.right5BallAuto);
        m_chooser.addOption("Center Balance", m_autos.centerBalanceAuto);
        m_chooser.addOption("Center Right 2 Ball", m_autos.centerRight2BallAuto);
        m_chooser.addOption("Center Left 2 Ball", m_autos.centerLeft2BallAuto);
        m_chooser.addOption("Right Side 3 Balance", m_autos.rightSide3BalanceAuto);
        m_chooser.addOption("Left Side 3 Balance", m_autos.leftSide3BalanceAuto);
        m_chooser.addOption("Right Side 2 Balance", m_autos.rightSide2BalanceAuto);
        m_chooser.addOption("Left Side 2 Balance", m_autos.leftSide2BalanceAuto);

        SmartDashboard.putData("Auto Mode", m_chooser);
        
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}