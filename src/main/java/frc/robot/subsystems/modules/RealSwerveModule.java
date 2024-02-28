package frc.robot.subsystems.modules;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import lib.SwerveModule;

public class RealSwerveModule implements AutoCloseable, SwerveModule {
    
    private final TalonFX m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final CANcoder m_absoluteEncoder;
    private final boolean kAbsoluteEncoderReversed;

    private final PIDController turningPidController;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public RealSwerveModule(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {
        
        m_turningMotor = new CANSparkMax(kTurningMotorId, MotorType.kBrushless);
        m_turningMotor.setInverted(turningMotorReversed);
                
        m_driveMotor = new TalonFX(kDriveMotorId, "rio");
        TalonFXConfigurator configurator = m_driveMotor.getConfigurator();
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withRotorToSensorRatio(DriveConstants.kDriveEncoderPositionConversionFactor / (DriveConstants.kWheelDiameterMeters * Math.PI));
        configurator.apply(feedbackConfigs);
        m_driveMotor.setInverted(driveMotorReversed);

        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = new CANcoder(absoluteEncoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders(); // Resets encoders every time the robot boots up
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        double desiredTurnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
        m_turningMotor.set(desiredTurnSpeed);

        double normalizedSpeed = state.speedMetersPerSecond / ModuleConstants.kMaxModuleSpeedMPS;
        m_driveMotor.set(normalizedSpeed);

        desiredState = state;
        SmartDashboard.putString("Swerve[" + m_absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    @Override
    public void resetEncoders() {
        m_driveMotor.setPosition(0);
    }

    public double getAbsoluteEncoderRotations() {
        double angle = m_absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Returns percent of a full rotation
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java
    }

    public double getAbsoluteEncoderRad() {
        double angle = m_absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Returns percent of a full rotation
        angle = Units.rotationsToRadians(angle);
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * @return The position of the drive motor in meters
     */
    public double getDrivePosition() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    /**
     * @return The velocity of the drive motor in meters per second
    */
    public double getDriveVelocity() {
        return m_driveMotor.getVelocity().getValueAsDouble();
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
