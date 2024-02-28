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

    private final PIDController m_intakeAnglePidController;

    public Intake() {
        m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorId, MotorType.kBrushless);
        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorId, MotorType.kBrushless);

        m_angleEncoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderId);

        m_intakeAnglePidController = new PIDController(0.05, 0, 0);
        m_intakeAnglePidController.setTolerance(IntakeConstants.kIntakeAngleToleranceDegrees);
    }

    /**
     * updates intake angle with pid loops
     */
    public void updateIntake() {
        m_intakeAngleMotor.set(m_intakeAnglePidController.calculate(m_angleEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    /**
     * runs intake and indexer motors at intake speed
     */
    public void intakeNote() {
        m_intakeMotor.set(IntakeConstants.kIntakeRollerSpeedPercent);
        m_indexerMotor.set(IntakeConstants.kIndexerIntakeSpeedPercent);
    }

    /**
     * runs indexer at shooting speed
     */
    public void shootNote() {
        m_indexerMotor.set(IntakeConstants.kIndexerShootingSpeedPercent);
    }

    /**
     * stop the indexer motors
     */
    public void stopIndexer() {
        m_indexerMotor.stopMotor();
    }
    
    /**
     * raises intake to angle set in constants
     */
    public void raiseIntake() {
        m_intakeAnglePidController.setSetpoint(IntakeConstants.kIntakeRaisedAngleDegrees);
    }
    
    /**
     * lowers intake to angle set in constants 
     * sets the intake and indexer  motors to max speed
     */
    public void lowerAndRunIntake() {
        m_intakeAnglePidController.setSetpoint(IntakeConstants.kIntakeLoweredAngleDegrees);
        m_intakeMotor.set(IntakeConstants.kIntakeRollerSpeedPercent);
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
        stopIndexer();
        stopIntakeRollers();
        stopIntakeRotation();
    }
}
