import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;

    @BeforeEach
    void setup() {
    }

    @Test
    public void testGetDrivePosition() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
        // Act and Assert
        assertTrue(0 <= subsystem.getDrivePosition() && subsystem.getDrivePosition() <= 1000);
        System.out.println("Drive position = " + subsystem.getDrivePosition());
    }
}
