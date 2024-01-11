package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    // Auto Commands
    private final Command m_straightAuto = new StraightAutoCmd(m_swerveSubsystem);

    private final Command m_circleAuto = new CircleAutoCmd(m_swerveSubsystem);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                m_driverController.y()));

        configureBindings();

        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Straight Auto", m_straightAuto);

        m_chooser.addOption("Circle Auto", m_circleAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Auto Options").add(m_chooser);
    }

  private void configureBindings() {
    new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
  }
  
  public SwerveSubsystem getSwerveSubsystem() {
      return m_swerveSubsystem;
  }

  public Command getAutonomousCommand() {
      return m_chooser.getSelected();
  }  
}