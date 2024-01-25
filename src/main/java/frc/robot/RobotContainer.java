package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
    private final Swerve m_swerveSubsystem = new Swerve();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerveSubsystem);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerveSubsystem);

    private final Command m_straightPathAuto = new PathPlannerAuto("Test Auto");

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Field Oriented Chooser
    SendableChooser<Boolean> m_fieldOrientedChooser = new SendableChooser<>();

    public ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                true));

        configureBindings();

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Straight Auto", m_straightAuto);
        m_autoChooser.addOption("Circle Auto", m_circleAuto);
        m_autoChooser.addOption("Straight Path Auto", m_straightPathAuto);

        // Add options to the field oriented chooser
        m_fieldOrientedChooser.setDefaultOption("Field Oriented", true);
        m_fieldOrientedChooser.addOption("Robot Oriented", false);

        // Put the choosers on the dashboard
        mainTab.add(m_autoChooser);
        mainTab.add(m_fieldOrientedChooser);

        SmartDashboard.putData(m_swerveSubsystem);
        SmartDashboard.putData(m_straightAuto);
        SmartDashboard.putData(m_circleAuto);
        SmartDashboard.putData(m_swerveSubsystem.TurnToAngle(45));
    }

  private void configureBindings() {
    new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
  }
  
  public Swerve getSwerveSubsystem() {
      return m_swerveSubsystem;
  }

  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
  }  
}