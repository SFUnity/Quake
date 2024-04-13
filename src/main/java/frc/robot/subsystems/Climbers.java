package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climbers extends SubsystemBase{
    private final CANSparkMax m_climberMotorL, m_climberMotorR;
    private final SparkPIDController m_leftClimberPidController, m_rightClimberPidController;
    private final RelativeEncoder m_leftClimberEncoder, m_rightClimberEncoder;

    private ShuffleboardTab loggingTab = Shuffleboard.getTab("Logging");

    private GenericEntry climberLPositionEntry = loggingTab.addPersistent("ClimberL Position", 0).getEntry();
    private GenericEntry climberRPositionEntry = loggingTab.addPersistent("ClimberR Position", 0).getEntry();

    public Climbers() {
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorIdL, MotorType.kBrushless);
        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorIdR, MotorType.kBrushless);

        m_climberMotorL.setSecondaryCurrentLimit(80, 1);
        m_climberMotorR.setSecondaryCurrentLimit(80, 1);

        m_climberMotorL.setIdleMode(IdleMode.kBrake);
        m_climberMotorR.setIdleMode(IdleMode.kBrake);

        m_leftClimberEncoder = m_climberMotorL.getEncoder();
        m_rightClimberEncoder = m_climberMotorR.getEncoder();
        
        m_leftClimberPidController = m_climberMotorL.getPIDController();
        m_rightClimberPidController = m_climberMotorR.getPIDController();

        m_leftClimberPidController.setP(0.05);
        m_rightClimberPidController.setP(0.05);
    }

    @Override
    public void periodic() {
        climberLPositionEntry.setDouble(m_leftClimberEncoder.getPosition());
        climberRPositionEntry.setDouble(m_rightClimberEncoder.getPosition());
    }

    public void extend() {
        m_leftClimberPidController.setReference(ClimberConstants.kExtendHeightRotations, ControlType.kPosition);
        m_rightClimberPidController.setReference(ClimberConstants.kExtendHeightRotations, ControlType.kPosition);
    }

    public void retract() {
        m_leftClimberPidController.setReference(ClimberConstants.kRetractHeightRotationsL, ControlType.kPosition);
        m_rightClimberPidController.setReference(ClimberConstants.kRetractHeightRotationsR, ControlType.kPosition);
    }

    public Command defaultCmd() {
        return run(() -> retract());
    }

    public void stop() {
        m_climberMotorL.stopMotor();
        m_climberMotorR.stopMotor();
    }
}
