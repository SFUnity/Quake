package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {
    public final Limelight m_limelight = Limelight.getInstance();
    private final Swerve m_swerve = new Swerve();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    ControllerConstants.kDriverControllerId);            

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);
    // private final Command m_circleAuto = new CircleAutoCmd(m_swerve);
    private final Command m_autoAlign = new RunCommand(() -> {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, m_swerve.turnToTagSpeed(m_limelight.getTargetOffsetX()), m_swerve.getRotation2d());
            chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

            SwerveModuleState[] moduleStates = 
            DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            m_swerve.setModuleStates(moduleStates);
        }, m_swerve).until(() -> m_limelight.alignedWithTag());
    // private final Command m_testShoot = new RunCommand(() -> {
    //         m_shooter.readyShootAmp();
    //         m_shooter.setAngleMotorSpeeds();
    //         m_shooter.setFlywheelMotorSpeed();
    //     }, m_shooter).withTimeout(0.5).andThen(m_shooter.putNoteIntoFlywheelsCommand());

    // private final Command m_straightPath;
    private final Command m_swervyPath;

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Field Oriented Chooser
    SendableChooser<Boolean> m_fieldOrientedChooser = new SendableChooser<>();

    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");                                                

    public RobotContainer() {
        m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                m_limelight,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                m_driverController.leftBumper(),
                m_driverController.b(),
                m_driverController.povDown(),
                m_driverController.povUp(),
                m_driverController.povLeft(),
                m_driverController.povRight()));

        configureBindings();
                
        // m_straightPath = new PathPlannerAuto("Straight Path Auto");
        m_swervyPath = new PathPlannerAuto("Swervy Path Auto");

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}, m_swerve));
        m_autoChooser.addOption("Auto Align", m_autoAlign);
        // m_autoChooser.addOption("Straight Path", m_straightPath);
        m_autoChooser.addOption("Swervy Path", m_swervyPath);
        m_autoChooser.addOption("Straight Auto", m_straightAuto);
        // m_autoChooser.addOption("Circle Auto", m_circleAuto);

        // Add options to the field oriented chooser
        m_fieldOrientedChooser.setDefaultOption("Field Oriented", true);
        m_fieldOrientedChooser.addOption("Robot Oriented", false);

        // Put the choosers on the dashboard
        driversTab.add(m_autoChooser)
                  .withSize(2, 1)
                  .withPosition(0, 0);
        driversTab.add(m_fieldOrientedChooser)
                  .withSize(2, 1)
                  .withPosition(0, 2);

        SmartDashboard.putData(m_swerve);
    }
  
    private void configureBindings() {
        new Trigger(m_driverController.x()).whileTrue(m_swerve.SetXCommand());
        new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetHeading()));
    }

    public Swerve getSwerve() {
        return m_swerve;
    }

    public Limelight getLimelight() {
        return m_limelight;
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }  
}