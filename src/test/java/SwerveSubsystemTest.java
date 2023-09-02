import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.subsystems.SwerveSubsystem;

public class SwerveSubsystemTest {
    SwerveSubsystem subsystem;

    @Mock Pigeon2 gyroMock;

    @BeforeEach
    void setup() {
        // Arrange
        MockitoAnnotations.openMocks(this);
        
        // Put stubs here
        
        subsystem = new SwerveSubsystem(gyroMock);
    }

    @Test
    void testGetHeadingLarge() {
        // Arrange
        when(gyroMock.getYaw()).thenReturn(1000.0);
        // Act and Assert
        assertTrue(subsystem.getHeading() >= -360 && subsystem.getHeading() <= 360);
    }

    @Test
    void testGetHeadingSmall() {
        // Arrange
        when(gyroMock.getYaw()).thenReturn(1.0);
        // Act and Assert
        assertTrue(subsystem.getHeading() >= -360 && subsystem.getHeading() <= 360);
    }

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
