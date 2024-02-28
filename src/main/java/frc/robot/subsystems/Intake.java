package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final CANSparkMax m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorId, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorId, MotorType.kBrushless);
    private final CANSparkMax m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorId, MotorType.kBrushless);
    
    private final CANcoder m_angleEncoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderId);

    private final PIDController m_intakeAnglePidController = new PIDController(0.05, 0, 0); //mess around with this later

    public Intake() {
        
    }

    /**
     * updates intake angle with pid loops
     * called periodically from IntakeControllerCmd
     */
    public void updateIntake() {
        moveIntake(m_intakeAnglePidController.calculate(m_angleEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    /**
     * rotates intake at specified speed
     * @param speed -1 to 1, speed as a percentage of max speed
     */
    public void moveIntake(double speed) {
        m_intakeAngleMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    }

    /**
     * runs intake and indexer at specified speed
     * @param speed -1 to 1, speed as a percentage of max speed
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(Math.max(-1, Math.min(1, speed)));
        m_indexerMotor.set(Math.max(-1, Math.min(1, speed)));
    }

    public void runIndexer(double speed) {
        m_indexerMotor.set(Math.max(-1, Math.min(1, speed)));
    }

    /**
     * sets the angle variable to a certain value which will cause the pid loops to rotate the intake to that angle
     * @param angle radians
     */
    public void setIntakeToAngle(double angle) {
        m_intakeAnglePidController.setSetpoint(angle);
    }

    /**
     * start running the intake and indexer  motors
     */
    public void startIntake() {
        runIntake(1);
    }

    public void startIndexer() {
        runIndexer(1);
    }

    /**
     * stop the intake motors
     */
    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    /**
     * stop the indexer motors
     */
    public void stopIndexer() {
        m_indexerMotor.stopMotor();
    }

    /**
     * stop intake and indexer motors
     */
    public void stopAll() {
        stopIntake();
        stopIndexer();
    }

    /**
     * stop the intake rotation motor
     */
    public void stopIntakeRotation() {
        m_intakeAngleMotor.stopMotor();
    }

    /**
     * lowers intake to angle set in constants
     */
    public void lowerIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeLoweredAngleRadians);
    }
    
    /**
     * raises intake to angle set in constants
     */
    public void raiseIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngleRadians);
    }
    
    /**
     * lowers intake to angle set in constants 
     * sets the intake and indexer  motors to max speed
     */
    public void lowerAndRunIntake() {
        lowerIntake();
        startIntake();
    }
    
    /**
     * raises intake to angle set in constants 
     * turns the intake  motor off, but doesn't affect indexer  motor
     */
    public void raiseAndStopIntake() {
        raiseIntake();
        stopIntake();
    }
}
