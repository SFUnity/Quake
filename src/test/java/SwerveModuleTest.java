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

    /**
     * @param speed in m/s
     * @param angle in degrees
     */
    void setStateTemplate(double speed, double angle) {
        // Arrange
        double fakeTurningPosition = 0.5;
        when(subsystem.getTurningPosition()).thenReturn(fakeTurningPosition);
        double angleInRadians = angle * Math.PI / 180;
        SwerveModuleState expectedState = new SwerveModuleState(speed, new Rotation2d(angleInRadians));
        // Act
        subsystem.setState(expectedState);
        // Assert                   Sets the drive speed to be proportional to the max speed
        verify(mockDriveMotor).set(expectedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // verify(mockTurningMotor).set();
    }

    @Test
    void testStraigtForwardFullThrottle() {
        setStateTemplate(1.0, 0);
    }

    @Test
    void test90InPlace() {
        setStateTemplate(0.0, 90);
    }

    @Test
    void test180InPlace() {
        setStateTemplate(0.0, 180);
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
