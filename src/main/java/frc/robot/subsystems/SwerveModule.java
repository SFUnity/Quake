package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final AnalogInput m_absoluteEncoder;
    private final double kAbsoluteEncoderOffsetRad;
    private final boolean kAbsoluteEncoderReversed;

    private final PIDController turningPidController;
    
    public SwerveModule(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
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
}
