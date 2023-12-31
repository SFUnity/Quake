package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    private final SwerveSubsystem m_swerveSubsystem;

    private final CommandXboxController m_driverController = new CommandXboxController(
                    OperatorConstants.kDriverControllerPort);

    private final CommandFactory commandFactory;

    private final Command autonomousCommand;

    public RobotContainer(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;

        commandFactory = new CommandFactory(m_swerveSubsystem);

        autonomousCommand = commandFactory.AutoPath(
          "Test Path",
          new PathConstraints(3, 2),
          null
        );

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

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
