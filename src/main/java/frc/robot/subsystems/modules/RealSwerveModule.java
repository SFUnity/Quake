package frc.robot.subsystems.modules;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import lib.SwerveModule;

public class RealSwerveModule implements AutoCloseable, SwerveModule {
    private final TalonFX m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final CANcoder m_absoluteEncoder;
    private final boolean kAbsoluteEncoderReversed;

    private final PIDController turningPidController;
    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public RealSwerveModule(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {
        
        m_turningMotor = new CANSparkMax(kTurningMotorId, MotorType.kBrushless);
        m_turningMotor.setInverted(turningMotorReversed);
        m_turningMotor.setSecondaryCurrentLimit(80, 1);
                
        m_driveMotor = new TalonFX(kDriveMotorId, "rio");
        TalonFXConfigurator configurator = m_driveMotor.getConfigurator();
        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(DriveConstants.kDriveEncoderPositionConversionFactor / (DriveConstants.kWheelDiameterMeters * Math.PI));
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.02; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.005; // no output for error derivative
        // configurator.apply(feedbackConfigs);
        configurator.apply(slot0Configs);
        m_driveMotor.setInverted(driveMotorReversed);

        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = new CANcoder(absoluteEncoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); // Consider adding the kI & kD
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders(); // Resets encoders every time the robot boots up
    }

    public double getKrakenSupplyVoltage() {
        return m_driveMotor.getSupplyVoltage().getValueAsDouble();
    }

    public double getKrakenSupplyCurrent() {
        return m_driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getTurningSupplyVoltage() {
        return m_turningMotor.getBusVoltage();
    }

    public double getTurningOutputCurrent() {
        return m_turningMotor.getOutputCurrent();
    }

    public void applyConfigs(double kS, double kV, double kP, double kI, double kD) {
        TalonFXConfigurator configurator = m_driveMotor.getConfigurator();
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI; 
        slot0Configs.kD = kD; 
        configurator.apply(slot0Configs);
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

        m_driveMotor.setControl(m_driveRequest.withVelocity(state.speedMetersPerSecond / (DriveConstants.kWheelDiameterMeters * Math.PI) * DriveConstants.kDriveEncoderPositionConversionFactor));

        desiredState = state;
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
