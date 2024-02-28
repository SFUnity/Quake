package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final CANSparkMax m_intakeAngleMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_indexerMotor;
    
    private final CANcoder m_angleEncoder;

    private final PIDController m_anglePidController;

    public Intake() {
        m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorId, MotorType.kBrushless);
        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorId, MotorType.kBrushless);

        m_angleEncoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderId);

        m_anglePidController = new PIDController(0.05, 0, 0);
        m_anglePidController.setTolerance(IntakeConstants.kIntakeAngleToleranceDegrees);
    }

    /**
     * updates intake angle with pid loops
     */
    public void setAngleMotorSpeeds() {
        m_intakeAngleMotor.set(m_anglePidController.calculate(m_angleEncoder.getAbsolutePosition().getValueAsDouble()));
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
        m_anglePidController.setSetpoint(IntakeConstants.kIntakeLoweredAngleDegrees);
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
        m_anglePidController.setSetpoint(IntakeConstants.kIntakeRaisedAngleDegrees);
        m_intakeMotor.stopMotor();
    }
}
