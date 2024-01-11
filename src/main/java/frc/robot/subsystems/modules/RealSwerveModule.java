package frc.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import lib.SwerveModule;

public class RealSwerveModule implements AutoCloseable, SwerveModule {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final AnalogInput m_absoluteEncoder;
    private final double kAbsoluteEncoderOffset;
    private final boolean kAbsoluteEncoderReversed;

    private final PIDController turningPidController;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public RealSwerveModule(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        m_driveMotor = new CANSparkMax(kDriveMotorId, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(kTurningMotorId, MotorType.kBrushless);

        m_driveMotor.setInverted(driveMotorReversed);
        m_turningMotor.setInverted(turningMotorReversed);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        kAbsoluteEncoderOffset = absoluteEncoderOffset;
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = new AnalogInput(absoluteEncoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders(); // Resets encoders every time the robot boots up
    }

    // ! For testing purposes only
    public RealSwerveModule(CANSparkMax driveMotor, CANSparkMax turningMotor, 
            AnalogInput absoluteEncoder, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
       
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();
        
        kAbsoluteEncoderOffset = absoluteEncoderOffset;
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = absoluteEncoder;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        double desiredTurnSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        m_turningMotor.set(desiredTurnSpeed);
        m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        desiredState = state;
        SmartDashboard.putString("Swerve[" + m_absoluteEncoder.getChannel() + "] state", state.toString());
    }

    @Override
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turningEncoder.setPosition(getAbsoluteEncoderRotations());
    }

    public double getAbsoluteEncoderRotations() {
        double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V(); // Returns percent of a full rotation
        angle -= kAbsoluteEncoderOffset;
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java

    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
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

    @Override
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
