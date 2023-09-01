import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

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

        subsystem = new SwerveModule(mockDriveMotor, mockTurningMotor, 
                absoluteEncoder, 0, false);
    }

    @Test
    public void testResetEncoders() {
        // Act
        subsystem.resetEncoders();
        // Assert
        assertEquals(0.0, subsystem.getDrivePosition());
        assertEquals(subsystem.getAbsoluteEncoderRad(), subsystem.getTurningPosition());
    }

    @Test
    public void testForwardDrive() {
        // Act
        subsystem.setDesiredState(new SwerveModuleState(0.5, new Rotation2d(0)));
        // Assert
        verify(mockDriveMotor).set(0.5 / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        verify(mockTurningMotor).set(0.0);
    }

    @Test
    public void testGetState() {
        // Arrange
        when(mockTurningEncoder.getPosition()).thenReturn(0.5);
        when(mockDriveEncoder.getVelocity()).thenReturn(0.5);
        // Act and Assert
        SwerveModuleState state = new SwerveModuleState(0.5, new Rotation2d(0.5));
        assertEquals(state, subsystem.getState());
        System.out.println(subsystem.getState().toString());
        System.out.println(state.toString());
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
