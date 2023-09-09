package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule implements AutoCloseable {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final AnalogInput m_absoluteEncoder;
    private final double kAbsoluteEncoderOffsetRad;
    private final boolean kAbsoluteEncoderReversed;

    private final PIDController turningPidController;
    
    /**
     * @param drive motor id
     * @param turning motor id
     * @param if drive motor is reversed
     * @param if turning motor is reversed
     * @param absolute encoder id
     * @param absolute encoder offset in radians
     * @param if absolute encoder is reversed
     */
    public MAXSwerveModule(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        m_driveMotor = new CANSparkMax(kDriveMotorId, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(kTurningMotorId, MotorType.kBrushless);

        m_driveMotor.setInverted(driveMotorReversed);
        m_turningMotor.setInverted(turningMotorReversed);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        kAbsoluteEncoderOffsetRad = absoluteEncoderOffset;
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = new AnalogInput(absoluteEncoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders(); // Resets encoders every time the robot boots up
    }

    /**
     * @param driveMotor
     * @param turningMotor
     * @param driveEncoder
     * @param turningEncoder
     * @param absoluteEncoder
     * @param absoluteEncoderOffset in radians
     * @param if the absoluteEncoder is reversed
     */
    // For testing purposes only
    public MAXSwerveModule(CANSparkMax driveMotor, CANSparkMax turningMotor, 
            AnalogInput absoluteEncoder, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
       
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();
        
        kAbsoluteEncoderOffsetRad = absoluteEncoderOffset;
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = absoluteEncoder;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return m_turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return m_turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V(); // Returns percent of a full rotation
        angle *= 2.0 * Math.PI; // convert to radians
        angle -= kAbsoluteEncoderOffsetRad;
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * @param desired swerve module state
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            m_driveMotor.set(0);
        } else {
            m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double desiredTurnSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        if (Math.abs(desiredTurnSpeed) < 0.001) {
            m_turningMotor.set(0);
        } else {
            m_turningMotor.set(desiredTurnSpeed);
        }

        SmartDashboard.putString("Swerve[" + m_absoluteEncoder.getChannel() + "] state", state.toString());
        System.out.println("Swerve[" + m_absoluteEncoder.getChannel() + "] state " + state.toString());
    }

    public void stopMotors() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }

    @Override
    public void close() throws Exception {
        m_driveMotor.close();
        m_turningMotor.close();
        m_absoluteEncoder.close();
    }
}
