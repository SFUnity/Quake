import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import static org.mockito.Mockito.*;

public class SwerveJoystickCmdTest {

    SwerveSubsystem subsystem;
    SwerveJoystickCmd command;
    
    @BeforeEach
    public void setup() {
        // Arrange
        subsystem = mock(SwerveSubsystem.class);
    }

    @Test
    public void testEnd() {
        // Arrange
        command = new SwerveJoystickCmd(subsystem, null, null, null, null);
        // Act
        command.end(false);
        // Assert
        verify(subsystem).zeroHeading();
    }
}
