import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;

    private void testAbsoluteEncoderTemplate(double offset, boolean reversed) {
        // Arrange
        subsystem = new SwerveModule(0, 1, 
            false, false, 2, 
            offset, reversed);
        // Act and Assert
        System.out.println(subsystem.getAbsoluteEncoderRad());
        assertEquals(offset * (reversed ? -1.0 : 1.0), subsystem.getAbsoluteEncoderRad());
    }

    @Nested
    public class TestsWithBasicSwerveModule {
        @BeforeEach
        void setup() {
            // Arrange
            subsystem = new SwerveModule(0, 1, 
                false, false, 2, 
                0, false);
        }

        @Test
        public void testResetEncoders() {
            // Act and Assert
            subsystem.resetEncoders();
            assertEquals(0, subsystem.getDrivePosition());
            assertEquals(subsystem.getAbsoluteEncoderRad(), subsystem.getTurningPosition());
        }

        @Test
        public void testGetState() {
            // Act and Assert
            assertEquals(new SwerveModuleState(subsystem.getDriveVelocity(), new Rotation2d(subsystem.getTurningPosition())), subsystem.getState());
        }

        @Test
        public void testGetDrivePosition() {
            // Act and Assert
            System.out.println("Drive position = " + subsystem.getDrivePosition());
            assertTrue(subsystem.getDrivePosition() >= 0);
        }

        @Test
        public void testGetTurningPosition() {
            // Act and Assert
            System.out.println("Drive position = " + subsystem.getTurningPosition());
            assertTrue(subsystem.getTurningPosition() >= 0);
        }
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
