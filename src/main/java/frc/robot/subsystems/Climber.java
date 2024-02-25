package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase{
    private final CANSparkMax m_climberMotorL, m_climberMotorR;
    private final PIDController m_climberControllerPID;
    private final RelativeEncoder m_climberEncoderL, m_climberEncoderR;
    private double speed, position = 0.0;

    public Climber() {
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorPortL, MotorType.kBrushless);
        m_climberEncoderL = m_climberMotorL.getEncoder();
        m_climberEncoderL.setPositionConversionFactor(ClimberConstants.kClimberDistanceConversionRate);

        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorPortR, MotorType.kBrushless);
        m_climberEncoderR = m_climberMotorR.getEncoder();
        m_climberEncoderR.setPositionConversionFactor(ClimberConstants.kClimberDistanceConversionRate);
        
        m_climberControllerPID = new PIDController(0.5, 0, 0);
    }

    public void updateClimber() {
        position = (Math.abs(m_climberEncoderL.getPosition()) + Math.abs(m_climberEncoderR.getPosition())) / 2;
        speed = m_climberControllerPID.calculate(position);

        m_climberMotorL.set(speed);
        m_climberMotorR.set(-speed);
    }

    public void climb() {
        m_climberControllerPID.setSetpoint(ClimberConstants.kClimbHeight);
    }

    public void decend() {
        m_climberControllerPID.setSetpoint(0.0);
    }

    public void resetEncoders() {
        m_climberEncoderL.setPosition(0.0);
        m_climberEncoderR.setPosition(0.0);
        position = 0.0;
    }

    public void stopAll() {
        m_climberMotorL.stopMotor();
        m_climberMotorR.stopMotor();
    }

}
