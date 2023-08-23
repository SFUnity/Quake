import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.*;

import java.util.function.Supplier;

public class SwerveJoystickCmdTest {

    @Mock SwerveSubsystem subsystem;
    CommandXboxController controller;
    SwerveJoystickCmd command;

    @Mock private Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    @Mock private Trigger fieldOrientedFunction;

    @BeforeEach
    public void setup() {
        // Arrange
        MockitoAnnotations.openMocks(this);

        controller = new CommandXboxController(0);
        
        when(xSpdFunction.get()).thenReturn(0.5);
        when(ySpdFunction.get()).thenReturn(0.5);
        when(turningSpdFunction.get()).thenReturn(0.5);
        when(fieldOrientedFunction.getAsBoolean()).thenReturn(false);

        command = spy(new SwerveJoystickCmd(
                subsystem,
                xSpdFunction, 
                ySpdFunction, 
                turningSpdFunction,
                fieldOrientedFunction));
    }

    @Test
    public void testExecuteApproximate() {
        // Act
        command.execute();
        // Assert
        verify(command).applyDeadBandXSpeed(anyDouble());
        verify(command).applyDeadBandYSpeed(anyDouble());
        verify(command).applyDeadBandTurningSpeed(anyDouble());

        verify(command).smoothXSpeed(anyDouble());
        verify(command).smoothYSpeed(anyDouble());
        verify(command).smoothTurningSpeed(anyDouble());

        verify(command).speedsToChassisSpeeds(anyDouble(), anyDouble(), anyDouble());
        
        verify(subsystem).setModuleStates(any(SwerveModuleState[].class));
    }

    @Test
    public void testExecuteExact() {
        // Act
        command.execute();
        // Assert
        verify(command).applyDeadBandXSpeed(anyDouble());
        verify(command).applyDeadBandYSpeed(anyDouble());
        verify(command).applyDeadBandTurningSpeed(anyDouble());

        verify(command).smoothXSpeed(anyDouble());
        verify(command).smoothYSpeed(anyDouble());
        verify(command).smoothTurningSpeed(anyDouble());

        verify(command).speedsToChassisSpeeds(anyDouble(), anyDouble(), anyDouble());
        
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
