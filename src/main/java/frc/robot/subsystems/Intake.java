package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_intakeRollerMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorPort, MotorType.kBrushless);
    
    private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    private final PIDController m_intakePID = new PIDController(0.05, 0, 0); //mess around with this later

    private final Rev2mDistanceSensor distOnboard;
    private double desiredAngle = 0.0;

    private final double toleranceDegrees = 1.0;

    public Intake() {
        // add port
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);
        
        desiredAngle = IntakeConstants.kIntakeRaisedAngleRadians;
    }

    /**
     * command to run the update intake function
     * placeholder since intake will have a customized default command
     */
    public Command runUpdateIntake() {
        return run(() -> this.updateIntake());
    }

    /**
     * updates intake angle with pid loops
     * called periodically from IntakeControllerCmd
     */
    public void updateIntake() {
        if (Math.abs(m_encoder.getAbsolutePosition().getValueAsDouble() - desiredAngle) > toleranceDegrees) {
            moveIntake(m_intakePID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble(), desiredAngle) / IntakeConstants.kTurningMotorMaxSpeed);
        } else {
            stopIntakeRotation();
        }
    }

    /**
     * returns a boolean based on if there is currently a note in the indexer or not
     */
    public boolean noteInIndexer() {
        return distOnboard.isRangeValid() && distOnboard.getRange() < IntakeConstants.kDistanceActivationThresholdMin;
    }

    /**
     * rotates intake at specified speed
     * @param speed -1 to 1, speed as a percentage of max speed
     */
    public void moveIntake(double speed) {
        m_intakeAngleMotor.set(speed / IntakeConstants.kIntakeAngleMotorMaxSpeed);
    }

    /**
     * runs intake and indexer at specified speed
     * @param speed -1 to 1, speed as a percentage of max speed
     */
    public void runIntake(double speed) {
        m_intakeRollerMotor.set(speed / IntakeConstants.kIntakeRollerMotorMaxSpeed);
        m_indexerMotor.set(speed / IntakeConstants.kIndexerMotorMaxSpeed);
    }

    /**
     * sets the angle variable to a certain value which will cause the pid loops to rotate the intake to that angle
     * @param desiredAngle radians
     */
    public void setIntakeToAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    /**
     * start running the intake and indexer  motors
     */
    public void startIntake() {
        runIntake(1);
    }

    /**
     * stop the intake motors
     */
    public void stopIntake() {
        m_intakeRollerMotor.stopMotor();
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
        setIntakeToAngle(IntakeConstants.kIntakeLoweredAngle);
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
        setIntakeToAngle(IntakeConstants.kIntakeLoweredAngle);
        startIntake();
    }
    
    /**
     * raises intake to angle set in constants 
     * turns the intake  motor off, but doesn't affect indexer  motor
     */
    public void raiseAndStopIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngleRadians);
        stopIntake();
    }
}
