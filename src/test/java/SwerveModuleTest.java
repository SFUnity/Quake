import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;

    CANSparkMax mockDriveMotor;
    CANSparkMax mockTurningMotor;

    RelativeEncoder mockDriveEncoder;
    RelativeEncoder mockTurningEncoder;

    AnalogInput absoluteEncoder;
    AnalogInputSim mockAbsoluteEncoder;

    @BeforeEach
    void setup() {
        // Arrange
        mockDriveMotor = mock(CANSparkMax.class);
        mockTurningMotor = mock(CANSparkMax.class);

        mockDriveEncoder = mock(RelativeEncoder.class);
        mockTurningEncoder = mock(RelativeEncoder.class);

        absoluteEncoder = new AnalogInput(0);
        mockAbsoluteEncoder = new AnalogInputSim(absoluteEncoder);

        when(mockDriveMotor.getEncoder()).thenReturn(mockDriveEncoder);
        when(mockTurningMotor.getEncoder()).thenReturn(mockTurningEncoder);

        when(mockDriveEncoder.getPosition()).thenReturn(0.0);
        when(mockTurningEncoder.getPosition()).thenReturn(0.0);

        when(mockDriveEncoder.getVelocity()).thenReturn(0.5);
        when(mockTurningEncoder.getVelocity()).thenReturn(0.5);

        subsystem = new SwerveModule(mockDriveMotor, mockTurningMotor, 
                absoluteEncoder, 0, false);
    }

    @Test
    public void testResetEncoders() {
        // Arrange                                                    percent of a full rotation ↓
        subsystem.setDesiredState(new SwerveModuleState(0.5, new Rotation2d(10 * 2.0 * Math.PI)));
        // Act
        subsystem.resetEncoders();
        // Assert
        assertEquals(0.0, subsystem.getDrivePosition());
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
