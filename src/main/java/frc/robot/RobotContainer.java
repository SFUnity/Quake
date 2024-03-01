package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.CircleAutoCmd;
// import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.StraightAutoCmd;
import frc.robot.commands.SwerveJoystickCmd;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TestingSubsystem;


public class RobotContainer {
    private final Swerve m_swerve = new Swerve();

    // private final LEDs m_LEDs = new LEDs();

    // private final Intake m_intake = new Intake();

    private final TestingSubsystem m_testingSubsystem = new TestingSubsystem();

    private final Shooter m_shooter = new Shooter();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    ControllerConstants.kDriverControllerId);
    private final CommandXboxController m_operationsController = new CommandXboxController(
                    ControllerConstants.kOperationControllerId);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerve);

    private final Command m_straightPathAuto;

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Field Oriented Chooser
    SendableChooser<Boolean> m_fieldOrientedChooser = new SendableChooser<>();

    public ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    public RobotContainer() {
        m_swerve.setDefaultCommand(new SwerveJoystickCmd(
                m_swerve,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                true));

        // m_shooter.setDefaultCommand(m_shooter.setToAngleCommand());
        m_shooter.setDefaultCommand(new ShooterCmd(
                m_shooter, 
                m_operationsController.x(), 
                m_operationsController.y(),
                m_operationsController.leftBumper(),
                m_operationsController.rightBumper()));
        
        // m_intake.setDefaultCommand(new IntakeCmd(
        //         m_intake,
        //         m_operationsController.a(), 
        //         m_operationsController.b()));

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
        mainTab.add(m_autoChooser);
        mainTab.add(m_fieldOrientedChooser);

        SmartDashboard.putData(m_swerve);
        SmartDashboard.putData(m_straightAuto);
        SmartDashboard.putData(m_circleAuto);
        SmartDashboard.putData(m_swerve.TurnToAngle(45));
    }

    private void configureBindings() {
        new Trigger(m_driverController.x()).whileTrue(m_swerve.SetXCommand());
        new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(2, 2, new Rotation2d(0)))).andThen(() -> m_swerve.resetHeading()));

        // new Trigger(() -> m_shooter.isNoteInShooter()).onTrue(new InstantCommand(m_intake::stopIndexer, m_intake));

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