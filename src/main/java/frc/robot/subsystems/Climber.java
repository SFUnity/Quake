package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase{
    private final CANSparkMax m_climberMotorL, m_climberMotorR;
    private final SparkPIDController m_leftClimberControllerPID, m_rightClimberControllerPID;
    private final RelativeEncoder m_climberEncoderL, m_climberEncoderR;
    private double speed, position = 0.0;

    public Climber() {
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorIdL, MotorType.kBrushless);
        m_climberEncoderL = m_climberMotorL.getEncoder();
        m_climberEncoderL.setPositionConversionFactor(ClimberConstants.kClimberDistanceConversionRate);

        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorIdR, MotorType.kBrushless);
        m_climberEncoderR = m_climberMotorR.getEncoder();
        m_climberEncoderR.setPositionConversionFactor(ClimberConstants.kClimberDistanceConversionRate);
        
        m_leftClimberControllerPID = m_climberMotorL.getPIDController();
        m_rightClimberControllerPID = m_climberMotorR.getPIDController();
    }

    // public void updateClimber() {
    //     position = (Math.abs(m_climberEncoderL.getPosition()) + Math.abs(m_climberEncoderR.getPosition())) / 2;
    //     speed = m_climberControllerPID.calculate(position);

    //     m_climberMotorL.set(speed);
    //     m_climberMotorR.set(speed);
    // }

    public void extend() {
        m_leftClimberControllerPID.setReference(ClimberConstants.kExtendHeight, ControlType.kPosition);
        m_rightClimberControllerPID.setReference(ClimberConstants.kExtendHeight, ControlType.kPosition);
    }

    public void retract() {
        m_leftClimberControllerPID.setReference(ClimberConstants.kRetractHeight, ControlType.kPosition);
        m_rightClimberControllerPID.setReference(ClimberConstants.kRetractHeight, ControlType.kPosition);
    }

    // public void resetEncoders() {
    //     m_climberEncoderL.setPosition(0.0);
    //     m_climberEncoderR.setPosition(0.0);
    //     position = 0.0;
    // }

    public void stopAll() {
        m_climberMotorL.stopMotor();
        m_climberMotorR.stopMotor();
    }

}
