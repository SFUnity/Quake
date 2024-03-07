package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.CircleAutoCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.StraightAutoCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
    private final Swerve m_swerve = new Swerve();

    // private final LEDs m_LEDs = new LEDs();
    private final Shooter m_shooter = new Shooter();

    private final Intake m_intake = new Intake(m_shooter);


    private final CommandXboxController m_driverController = new CommandXboxController(
                    ControllerConstants.kDriverControllerId);
    private final CommandPS5Controller m_operationsController = new CommandPS5Controller(
                    ControllerConstants.kOperationControllerId);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerve);

    private final Command m_straightPathAuto;

    // private final Command m_autoPath1;

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Field Oriented Chooser
    SendableChooser<Boolean> m_fieldOrientedChooser = new SendableChooser<>();

    public ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
    private GenericEntry intakeWorkingEntry = driversTab.add("Intake Working", true)
                                                        .withWidget(BuiltInWidgets.kToggleButton)
                                                        .withSize(3, 2)
                                                        .withPosition(2, 0)
                                                        .getEntry();

    public RobotContainer() {
        m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                m_driverController.leftBumper(),
                m_driverController.povUp(),
                m_driverController.povDown(),
                m_driverController.povRight(),
                m_driverController.povLeft(),
                /*
                 * povUp
                 * povDown
                 * povRight
                 * povLeft
                 */
                true));

        m_shooter.setDefaultCommand(new ShooterCmd(
                m_shooter, 
                m_operationsController.square(),
                m_operationsController.circle(),
                m_operationsController.L1(),
                m_operationsController.R1(),
                intakeWorkingEntry));
        
        m_intake.setDefaultCommand(new IntakeCmd(
                m_intake,
                m_operationsController.cross(),
                m_operationsController.triangle(),
                m_operationsController.square(),
                intakeWorkingEntry));

        NamedCommands.registerCommand("ampShoot", m_shooter.readyShootAmpCommand()); 
        NamedCommands.registerCommand("speakerShoot", m_shooter.readyShootSpeakerCommand());
        NamedCommands.registerCommand("putNoteInFlywheels", m_shooter.putNoteIntoFlywheelsCommand());

        m_straightPathAuto = new PathPlannerAuto("Test Auto");

        configureBindings();

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Straight Path Auto", m_straightPathAuto);
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
                  .withPosition(0, 3);

        SmartDashboard.putData(m_swerve);
        SmartDashboard.putData(m_intake);
        SmartDashboard.putData(m_shooter);
        SmartDashboard.putNumber("distance sensor", m_shooter.m_shooterDistanceSensor.getRange());
        // SmartDashboard.putData(m_straightAuto);
        // SmartDashboard.putData(m_circleAuto);
        // SmartDashboard.putData(m_swerve.TurnToAngle(45));
    }

    private void configureBindings() {
        new Trigger(m_driverController.x()).whileTrue(m_swerve.SetXCommand());
        new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(2, 2, new Rotation2d(0)))).andThen(() -> m_swerve.resetHeading()));

        // TODO test this
        new Trigger(() -> m_shooter.isNoteInShooter()).whileTrue(m_intake.noteInShooterCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // LED Triggers
        // new Trigger(() -> m_intake.noteInIndexer()).onTrue(m_LEDs.NoteInIndexerPattern());
        // new Trigger(() -> m_shooterDefaultCommand.noteInShooter()).onTrue(m_LEDs.NoteInShooterPattern());
        // new Trigger(() -> m_shooterDefaultCommand.shootingNote()).onTrue(m_LEDs.ShootingNotePattern());
    }

    public Swerve getSwerve() {
        return m_swerve;
    }

//     public LEDs getLEDs() {
//         return m_LEDs;
//     }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }  
}