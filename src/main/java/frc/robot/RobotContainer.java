package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                true));

        configureBindings();
    }

  private void configureBindings() {}

  // public Command getAutonomousCommand() {}
}
