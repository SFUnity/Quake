package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final CANSparkMax m_intakeAngleMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_indexerMotor;
    
    private final RelativeEncoder m_angleEncoder;

    private final SparkPIDController m_anglePidController;

    public Intake() {
        m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorId, MotorType.kBrushless);
        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorId, MotorType.kBrushless);

        //m_angleEncoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderId);
        m_angleEncoder = m_intakeAngleMotor.getEncoder();
        m_angleEncoder.setPositionConversionFactor((1/15)*(24/42)*(12/34)/360); // 1:15 gearbox, then a 24:42 and 12:34 gear reduction / 360 degrees

        m_anglePidController = m_intakeAngleMotor.getPIDController();
    }

    /**
     * stop the indexer motors
     */
    public void stopIndexer() {
        m_indexerMotor.stopMotor();
    }
    
    /**
     * lowers intake to angle set in constants 
     * sets the intake and indexer  motors to max speed
     */
    public void lowerAndRunIntake() {
        m_anglePidController.setReference(IntakeConstants.kIntakeLoweredAngleRevRotations, ControlType.kPosition);
        m_intakeMotor.set(IntakeConstants.kIntakeRollerSpeedPercent);
        m_indexerMotor.set(IntakeConstants.kIndexerIntakeSpeedPercent);
    }

    /**
     * stop the intake motors
     */
    public void stopIntakeRollers() {
        m_intakeMotor.stopMotor();
    }

    /**
     * stop the intake rotation motor
     */
    public void stopIntakeRotation() {
        m_intakeAngleMotor.stopMotor();
    }
    
    /**
     * raises intake to angle set in constants 
     * turns the intake  motor off, but doesn't affect indexer  motor
     */
    public void raiseAndStopIntake() {
        m_anglePidController.setReference(IntakeConstants.kIntakeRaisedAngleRevRotations, ControlType.kPosition);
        m_intakeMotor.stopMotor();
    }

    public Command noteInShooterCommand() {
        return run(() -> {
            stopIndexer();
            raiseAndStopIntake();
        });
    }
}
