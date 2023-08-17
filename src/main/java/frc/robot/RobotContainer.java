package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer implements AutoCloseable {
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                m_driverController.y()));

        configureBindings();
    }

  private void configureBindings() {
    new Trigger(m_driverController.a()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
  }

  // public Command getAutonomousCommand() {}

  @Override
  public void close() throws Exception {
    m_swerveSubsystem.close();
  }
}
