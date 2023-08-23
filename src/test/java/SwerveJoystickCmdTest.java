import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

public class SwerveJoystickCmdTest {

    SwerveSubsystem subsystem;
    SwerveJoystickCmd command;
    CommandXboxController controller;
    
    @BeforeEach
    public void setup() {
        // Arrange
        subsystem = mock(SwerveSubsystem.class);
        controller = new CommandXboxController(0);
        command = spy(new SwerveJoystickCmd(
                subsystem,
                () -> -controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                controller.y()));
    }

    @Test
    public void testExecute() {
        // Act
        command.execute();
        // Assert
        verify(command).applyDeadBandXSpeed(anyDouble());
        verify(command).applyDeadBandYSpeed(anyDouble());
        verify(command).applyDeadBandTurningSpeed(anyDouble());
        verify(subsystem).setModuleStates(any(SwerveModuleState[].class));
    }

    @Test
    public void testEnd() {
        // Act
        command.end(false);
        // Assert
        verify(subsystem).stopModules();
    }
}
