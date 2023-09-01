import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {

    SwerveModule subsystem;

    CANSparkMax mockDriveMotor;
    CANSparkMax mockTurningMotor;

    RelativeEncoder mockDriveEncoder;
    RelativeEncoder mockTurningEncoder;

    AnalogInput absoluteEncoder;
    AnalogInputSim simAbsoluteEncoder;

    private PIDController turningPidController;

    @BeforeEach
    void setup() {
        // Arrange
        mockDriveMotor = mock(CANSparkMax.class);
        mockTurningMotor = mock(CANSparkMax.class);

        mockDriveEncoder = mock(RelativeEncoder.class);
        mockTurningEncoder = mock(RelativeEncoder.class);

        absoluteEncoder = new AnalogInput(0);
        simAbsoluteEncoder = new AnalogInputSim(absoluteEncoder);

        when(mockDriveMotor.getEncoder()).thenReturn(mockDriveEncoder);
        when(mockTurningMotor.getEncoder()).thenReturn(mockTurningEncoder);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        subsystem = new SwerveModule(mockDriveMotor, mockTurningMotor, 
                absoluteEncoder, 0, false);
        Mockito.reset(mockDriveMotor, mockTurningMotor, mockDriveEncoder, mockTurningEncoder);
    }

    @Test
    public void testResetEncoders() {
        // Act
        subsystem.resetEncoders();
        // Assert
        verify(mockDriveEncoder).setPosition(0);
        verify(mockTurningEncoder).setPosition(subsystem.getAbsoluteEncoderRad());
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

    void setStateTemplate(double speed, double angle) {
        // Arrange
        when(subsystem.getTurningPosition()).thenReturn(0.5);
        SwerveModuleState expectedState = new SwerveModuleState(speed, new Rotation2d(angle * Math.PI / 180));
        // Act
        subsystem.setState(expectedState);
        // Arrange pt 2
        expectedState = SwerveModuleState.optimize(expectedState, subsystem.getState().angle);
        // Assert
        verify(mockDriveMotor).set(expectedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        verify(mockTurningMotor).set(turningPidController.calculate(0.5, expectedState.angle.getRadians()));
    }

    @Test
    void testStraigtForwardFullThrottle() {
        setStateTemplate(1.0, 0);
    }

    @Test
    void test180InPlace() {
        // setStateTemplate(0.0, 90);
        // Arrange
        when(subsystem.getTurningPosition()).thenReturn(0.5);
        SwerveModuleState expectedState = new SwerveModuleState(0.0, new Rotation2d(90 * Math.PI / 180));
        // Act
        subsystem.setState(expectedState);
        expectedState = SwerveModuleState.optimize(expectedState, subsystem.getState().angle);
        // Assert
        verify(mockDriveMotor).set(expectedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        verify(mockTurningMotor).set(turningPidController.calculate(0.5, expectedState.angle.getRadians()));
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
