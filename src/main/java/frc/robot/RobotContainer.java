package frc.robot;

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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CircleAutoCmd;
import frc.robot.commands.StraightAutoCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
    private final Swerve m_swerve = new Swerve();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerve);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerve);

    private final Command m_straightPathAuto = new PathPlannerAuto("Test Auto");

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
    new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(2, 2, new Rotation2d(0)))).andThen(() -> m_swerve.resetHeading()));
  }
  
  public Swerve getSwerve() {
      return m_swerve;
  }

  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
  }  
}