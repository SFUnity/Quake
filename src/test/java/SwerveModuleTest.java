import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
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
        Mockito.reset(mockDriveEncoder, mockTurningEncoder);
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
        when(subsystem.getTurningPosition()).thenReturn(0.0);
        double angleInRadians = angle * Math.PI / 180;
        SwerveModuleState expectedState = new SwerveModuleState(speed, new Rotation2d(angleInRadians));
        // Act
        subsystem.setState(expectedState);
        // Assert                   Sets the drive speed to be proportional to the max speed
        verify(mockDriveMotor).set(expectedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // Verifies that the speed of the turning motor is appropriate given optimization
        ArgumentCaptor<Double> argumentCaptor = ArgumentCaptor.forClass(Double.class);
        verify(mockTurningMotor).set(argumentCaptor.capture());
        double turningMotorSpeed = argumentCaptor.getValue();
        if (angle > 0 && angle <= 90 || angle > 180 && angle <= 270) {
            if (turningMotorSpeed <= 0.0 || turningMotorSpeed > 1.0) {
                throw new AssertionError("Value out of range!");
            }
        } else if (angle > 90 && angle < 180 || angle > 270 && angle < 360) {
            if (turningMotorSpeed >= 0.0 || turningMotorSpeed < -1.0) {
                throw new AssertionError("Value out of range!");
            }
        } else {
            if (turningMotorSpeed != 0.0) {
                throw new AssertionError("Value out of range!");
            }
        }
    }

    @Test
    void testNothing() {
        setStateTemplate(0, 0);
    }

    @Test
    void testStraigtForwardFullThrottle() {
        setStateTemplate(1.0, 0);
    }

    @Test
    void testStraigtBackwardFullThrottle() {
        setStateTemplate(-1.0, 0);
    }

    @Test
    void testEveryTenthAngleInPlace() {
        for (double i = 0.0; i <= 360; i += 10) {
            setStateTemplate(0.0, i);
            Mockito.reset(mockDriveMotor, mockTurningMotor);
        }
    }
    

    @AfterEach
    void shutdown() throws Exception {
        subsystem.close();
    }
}
