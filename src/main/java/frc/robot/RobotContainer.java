package frc.robot;

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
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    // Auto Commands Chooser
    private final Command m_straightAuto = new StraightAutoCmd(m_swerveSubsystem);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerveSubsystem);

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
                m_fieldOrientedChooser.getSelected()));

        configureBindings();

        // Add commands to the autonomous command chooser
        m_autoChooser.setDefaultOption("Straight Auto", m_straightAuto);

        m_autoChooser.addOption("Circle Auto", m_circleAuto);

        // Add options to the field oriented chooser
        m_fieldOrientedChooser.setDefaultOption("Field Oriented", true);
        m_fieldOrientedChooser.addOption("Robot Oriented", false);

        // Put the choosers on the dashboard
        mainTab.add(m_autoChooser);
        mainTab.add(m_fieldOrientedChooser);

        SmartDashboard.putData(m_swerveSubsystem);
        SmartDashboard.putData(m_straightAuto);
        SmartDashboard.putData(m_circleAuto);
    }

  private void configureBindings() {
    new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
  }
  
  public SwerveSubsystem getSwerveSubsystem() {
      return m_swerveSubsystem;
  }

  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
  }  
}