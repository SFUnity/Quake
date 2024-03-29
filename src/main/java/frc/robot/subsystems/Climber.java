package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase{
    private final CANSparkMax m_climberMotorL, m_climberMotorR;
    private final SparkPIDController m_leftClimberPidController, m_rightClimberPidController;

    public Climber() {
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorIdL, MotorType.kBrushless);
        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorIdR, MotorType.kBrushless);
        
        m_leftClimberPidController = m_climberMotorL.getPIDController();
        m_rightClimberPidController = m_climberMotorR.getPIDController();
    }

    public void extend() {
        m_leftClimberPidController.setReference(ClimberConstants.kExtendHeightRotations, ControlType.kPosition);
        m_rightClimberPidController.setReference(ClimberConstants.kExtendHeightRotations, ControlType.kPosition);
    }

    public void retract() {
        m_leftClimberPidController.setReference(ClimberConstants.kRetractHeightRotations, ControlType.kPosition);
        m_rightClimberPidController.setReference(ClimberConstants.kRetractHeightRotations, ControlType.kPosition);
    }

    public void stopAll() {
        m_climberMotorL.stopMotor();
        m_climberMotorR.stopMotor();
    }
}
