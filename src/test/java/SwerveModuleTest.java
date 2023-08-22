import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;

    @BeforeEach
    void setup() {
    }

    @Test
    public void testResetEncoders() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
        // Act and Assert
        subsystem.resetEncoders();
        assertEquals(subsystem.getDrivePosition(), 0);
        assertEquals(subsystem.getTurningPosition(), subsystem.getAbsoluteEncoderRad());
    }

    @Test
    public void testGetState() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
        // Act and Assert
        assertEquals(subsystem.getState(), new SwerveModuleState(subsystem.getDriveVelocity(), new Rotation2d(subsystem.getTurningPosition())));
    }

    @Test
    public void testGetDrivePosition() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
        // Act and Assert
        System.out.println("Drive position = " + subsystem.getDrivePosition());
        assertTrue(subsystem.getDrivePosition() >= 0);
    }

    @Test
    public void testGetTurningPosition() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
        // Act and Assert
        System.out.println("Drive position = " + subsystem.getTurningPosition());
        assertTrue(subsystem.getTurningPosition() >= 0);
    }
}
