import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;
    @BeforeEach
    void setup() {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            0, false);
    }

    @Test
    public void testResetEncoders() {
        // Act
        subsystem.resetEncoders();
        // Assert
        assertEquals(0, subsystem.getDrivePosition());
        assertEquals(subsystem.getAbsoluteEncoderRad(), subsystem.getTurningPosition());
    }

    public void testSetStateTemplate(double speedMetersPerSecond, double angle) {
        // Arrange
        angle *= 2.0 * Math.PI;
        SwerveModuleState goalState = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angle));
        // Act 
        subsystem.setDesiredState(goalState);
        // Assert
        assertEquals(speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond, subsystem.getDriveVelocity());
        assertTrue(-180 <= subsystem.getTurningPosition() / 360 && subsystem.getTurningPosition() / 360 <= 180);
    }

    @Test
    public void testGetState() {
        // Act and Assert
        assertEquals(new SwerveModuleState(subsystem.getDriveVelocity(), new Rotation2d(subsystem.getTurningPosition())), subsystem.getState());
    }

    @Test
    public void testGetDrivePosition() {
        // Act and Assert
        assertTrue(subsystem.getDrivePosition() >= 0);
    }

    @Test
    public void testGetTurningPosition() {
        // Act and Assert
        assertTrue(subsystem.getTurningPosition() >= 0);
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
