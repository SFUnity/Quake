import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveSubsystemTest {
    SwerveSubsystem subsystem;

    @Mock SwerveModule frontLeftMock;
    @Mock SwerveModule frontRightMock;
    @Mock SwerveModule backLeftMock;
    @Mock SwerveModule backRightMock;

    @Mock Pigeon2 gyroMock;

    @BeforeEach
    void setup() {
        // Arrange
        MockitoAnnotations.openMocks(this);

        // Put stubs here

        subsystem = new SwerveSubsystem(frontLeftMock, frontRightMock, 
                                        backLeftMock, backRightMock, gyroMock);
    }

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
