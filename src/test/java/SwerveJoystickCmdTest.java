import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

public class SwerveJoystickCmdTest {

    @Mock SwerveSubsystem subsystem;
    CommandXboxController controller;
    SwerveJoystickCmd command;

    @BeforeEach
    public void setup() {
        // Arrange
        MockitoAnnotations.openMocks(this);

        controller = new CommandXboxController(0);

        command = new SwerveJoystickCmd(
                subsystem,
                () -> -controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                controller.y());
    }

    @Test
    void negativeDeadbandToDeadbandShouldEqualZero() {
        for (double i = -1 * OperatorConstants.kDeadband; i < OperatorConstants.kDeadband; i += 0.01) {
            System.out.println(i);
            assertEquals(0.0, command.applyDeadBand(i));
        }
    }

    @Test
    void aboveDeadbandShouldStayTheSame() {
        for (double i = OperatorConstants.kDeadband + 0.01; i < 1; i += 0.01) {
            System.out.println(i);
            assertEquals(i, command.applyDeadBand(i));
        }
    }

    @Test
    void belowDeadbandShouldStayTheSame() {
        for (double i = -1 * OperatorConstants.kDeadband - 0.01; i > -1; i -= 0.01) {
            System.out.println(i);
            assertEquals(i, command.applyDeadBand(i));
        }
    }

    @Test
    void testEnd() {
        // Act
        command.end(false);
        // Assert
        verify(subsystem).stopModules();
    }
}
