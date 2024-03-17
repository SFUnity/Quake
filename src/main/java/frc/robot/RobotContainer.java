package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CircleAutoCmd;
import frc.robot.commands.StraightAutoCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {
    private final Swerve m_swerve = new Swerve();
    public final LimelightSubsystem m_limelight = LimelightSubsystem.getInstance();
    private final LEDs m_LEDs = new LEDs();
    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake(m_shooter);

    private final CommandXboxController m_driverController = new CommandXboxController(
                    ControllerConstants.kDriverControllerId);
    private final CommandPS5Controller m_operationsController = new CommandPS5Controller(
                    ControllerConstants.kOperationControllerId);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerve);

    private final Command m_4NoteSpeaker;
    private final Command m_sourceOut;
    private final Command m_ampOut;
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

    private ShuffleboardTab configsTab = Shuffleboard.getTab("Configs");                                                   

    public RobotContainer() {
        configsTab.add(m_swerve.setConfigsCommand()).withWidget(BuiltInWidgets.kCommand); 

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

        m_shooter.setDefaultCommand(new ShooterCmd(
                m_shooter, 
                m_limelight,
                m_operationsController.square(),
                m_operationsController.circle(),
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
                intakeWorkingEntry));

        NamedCommands.registerCommand("fullSpeakerShoot", new SequentialCommandGroup(m_shooter.readyShootSpeakerCommand(), m_shooter.putNoteIntoFlywheelsCommand(), m_shooter.stopShootingCommand()));
        NamedCommands.registerCommand("fullIntakeNote", new ParallelCommandGroup(m_shooter.intakeNoteCmd(), m_intake.lowerAndRunIntakeCmd()));
        NamedCommands.registerCommand("raiseAndStopIntake", m_intake.raiseAndStopIntakeCmd());
        NamedCommands.registerCommand("Straight", m_straightAuto);

        m_4NoteSpeaker = new PathPlannerAuto("4 Note Speaker");
        m_sourceOut = new PathPlannerAuto("Source Out");
        m_ampOut = new PathPlannerAuto("Amp Out");
        // m_straightPath = new PathPlannerAuto("Straight Path Auto");
        // m_swervyPath = new PathPlannerAuto("Swervy Path Auto");

        configureBindings();

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}, m_swerve, m_intake, m_shooter));
        m_autoChooser.addOption("4 Note Speaker", m_4NoteSpeaker);
        m_autoChooser.addOption("Source Out", m_sourceOut);
        m_autoChooser.addOption("Amp", m_ampOut);
        // m_autoChooser.addOption("Straight Path", m_straightPath);
        // m_autoChooser.addOption("Swervy Path", m_swervyPath);
        m_autoChooser.addOption("Straight Auto", m_straightAuto);
        m_autoChooser.addOption("Circle Auto", m_circleAuto);

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
        SmartDashboard.putData(new SequentialCommandGroup(m_shooter.readyShootSpeakerCommand(), m_shooter.putNoteIntoFlywheelsCommand(), m_shooter.stopShootingCommand()));
        SmartDashboard.putData("Turn to 45", m_swerve.TurnToAngle(45));
        SmartDashboard.putData("Turn to 0", m_swerve.TurnToAngle(0));
        SmartDashboard.putData("default LEDs", m_LEDs.defaultPattern());
        SmartDashboard.putData(m_LEDs.NoteInShooterPattern());
        SmartDashboard.putData("rainbow!", m_LEDs.setToRainbow());
    }
  
    private void configureBindings() {
        new Trigger(m_driverController.x()).whileTrue(m_swerve.SetXCommand());
        new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(2, 2, new Rotation2d(0)))).andThen(() -> m_swerve.resetHeading()));

        new Trigger(() -> m_shooter.isNoteInShooter()).whileTrue(m_intake.noteInShooterCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // LED Triggers
        new Trigger(() -> m_shooter.isNoteInShooter()).onTrue(m_LEDs.NoteInShooterPattern());
    }

    public Swerve getSwerve() {
        return m_swerve;
    }

    public LEDs getLEDs() {
        return m_LEDs;
    }
  
    public LimelightSubsystem getLimelightSubsystem() {
        return m_limelight; 
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }  
}