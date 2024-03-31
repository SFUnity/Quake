package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {
    public final LimelightSubsystem m_limelight = LimelightSubsystem.getInstance();
    private final Swerve m_swerve = new Swerve(m_limelight);
    private final LEDs m_LEDs = new LEDs();
    private final Shooter m_shooter = new Shooter(m_limelight);
    private final Intake m_intake = new Intake(m_shooter);

    private final CommandXboxController m_driverController = new CommandXboxController(
                    ControllerConstants.kDriverControllerId);
    private final CommandPS5Controller m_operationsController = new CommandPS5Controller(
                    ControllerConstants.kOperationControllerId);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);
    // private final Command m_circleAuto = new CircleAutoCmd(m_swerve);
    private final Command fullSpeakerShoot = new SequentialCommandGroup(m_shooter.readyShootSpeakerCommand(), m_shooter.putNoteIntoFlywheelsCommand()).alongWith(m_intake.raiseAndStopIntakeCmd());
    private final Command m_autoAlign = new RunCommand(() -> {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, m_swerve.turnToTagSpeed(), m_swerve.getRotation2d());
            chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

            SwerveModuleState[] moduleStates = 
            DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            m_swerve.setModuleStates(moduleStates);
        }, m_swerve).until(() -> m_limelight.alignedWithTag());
    private final Command m_autoShoot = m_autoAlign.alongWith(m_shooter.readyAutoShoot()).andThen(m_shooter.autoShoot());
    // private final Command m_testShoot = new RunCommand(() -> {
    //         m_shooter.readyShootAmp();
    //         m_shooter.setAngleMotorSpeeds();
    //         m_shooter.setFlywheelMotorSpeed();
    //     }, m_shooter).withTimeout(0.5).andThen(m_shooter.putNoteIntoFlywheelsCommand());

    private final Command m_justShootAndLeave;
    // private final Command m_straightPath;
    // private final Command m_swervyPath;

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Field Oriented Chooser
    SendableChooser<Boolean> m_fieldOrientedChooser = new SendableChooser<>();

    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
    private GenericEntry intakeWorkingEntry = driversTab.add("Intake Working", true)
                                                        .withWidget(BuiltInWidgets.kToggleButton)
                                                        .withSize(3, 2)
                                                        .withPosition(2, 0)
                                                        .getEntry();                                                  

    public RobotContainer() {
        m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                m_limelight,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                m_driverController.leftBumper(),
                m_driverController.b(),
                m_driverController.leftTrigger(0.05),
                m_driverController.povDown(),
                m_driverController.povUp(),
                m_driverController.povLeft(),
                m_driverController.povRight()));

        m_shooter.setDefaultCommand(new ShooterCmd(
                m_shooter, 
                m_operationsController.square(),
                m_operationsController.circle(),
                m_operationsController.triangle(),
                m_operationsController.L1(),
                m_operationsController.R1(),
                m_operationsController.L2(),
                m_operationsController.R2(),
                intakeWorkingEntry));
        
        m_intake.setDefaultCommand(new IntakeCmd(
                m_intake,
                m_operationsController.cross(),
                m_operationsController.triangle(),
                m_operationsController.square(),
                m_operationsController.circle(),
                intakeWorkingEntry));

        m_LEDs.setDefaultCommand(new LEDCmd(m_shooter, m_swerve, m_limelight, m_LEDs));

        NamedCommands.registerCommand("fullSpeakerShoot", fullSpeakerShoot);
        NamedCommands.registerCommand("readyAutoShoot", m_shooter.readyAutoShoot());
        NamedCommands.registerCommand("autoShoot", m_autoShoot);
        NamedCommands.registerCommand("fullIntakeNote", m_intake.lowerAndRunIntakeCmd().alongWith(m_shooter.intakeNoteCmd()));
        NamedCommands.registerCommand("shooterIntake", m_shooter.intakeNoteCmd());
        NamedCommands.registerCommand("raiseAndStopIntake", new WaitCommand(0.5).andThen(m_intake.raiseAndStopIntakeCmd()));
        NamedCommands.registerCommand("finishIntakingThenShoot", m_shooter.intakeNoteCmd().andThen(m_autoShoot));

        m_justShootAndLeave = new SequentialCommandGroup(m_shooter.readyShootSpeakerCommand(), m_shooter.putNoteIntoFlywheelsCommand(), new WaitCommand(5), m_straightAuto);
        // m_straightPath = new PathPlannerAuto("Straight Path Auto");
        // m_swervyPath = new PathPlannerAuto("Swervy Path Auto");

        configureBindings();

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}, m_swerve, m_intake, m_shooter));
        m_autoChooser.addOption("Center CBA1", new PathPlannerAuto("Center CBA1"));
        m_autoChooser.addOption("Center CBA2", new PathPlannerAuto("Center CBA2"));
        m_autoChooser.addOption("Center CB3", new PathPlannerAuto("Center CB3"));
        m_autoChooser.addOption("Source 43", new PathPlannerAuto("Source 43"));
        m_autoChooser.addOption("Source 53", new PathPlannerAuto("Source 53"));
        m_autoChooser.addOption("Amp A1", new PathPlannerAuto("Amp A1"));
        m_autoChooser.addOption("Amp 12", new PathPlannerAuto("Amp 12"));
        m_autoChooser.addOption("Just Shoot", fullSpeakerShoot);
        m_autoChooser.addOption("Just Shoot and Leave", m_justShootAndLeave);
        // m_autoChooser.addOption("Straight Path", m_straightPath);
        // m_autoChooser.addOption("Swervy Path", m_swervyPath);
        // m_autoChooser.addOption("Straight Auto", m_straightAuto);
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
        SmartDashboard.putData(m_intake);
        SmartDashboard.putData(m_shooter);
        SmartDashboard.putData(new SequentialCommandGroup(m_shooter.readyShootSpeakerCommand(), m_shooter.putNoteIntoFlywheelsCommand()));
        SmartDashboard.putData("rainbow!", m_LEDs.setToRainbow());
    }
  
    private void configureBindings() {
        new Trigger(m_driverController.x()).whileTrue(m_swerve.SetXCommand());
        new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(2, 2, new Rotation2d(0)))).andThen(() -> m_swerve.resetHeading()));

        new Trigger(m_operationsController.povUp()).onTrue(new InstantCommand(() -> m_limelight.setPipeline(0)));
        new Trigger(m_operationsController.povDown()).onTrue(new InstantCommand(() -> m_limelight.setPipeline(1)));

        // TODO test this once done with the other stuff
        // new Trigger(() -> m_shooter.isNoteInShooter() && DriverStation.isTeleop()).whileTrue(m_intake.noteInShooterCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    public Swerve getSwerve() {
        return m_swerve;
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }  
}