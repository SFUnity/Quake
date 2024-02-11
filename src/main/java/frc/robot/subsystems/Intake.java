package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final CANSparkMax m_IntakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IntakeFlywheelMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IndexerFlywheelMotor = new CANSparkMax(IntakeConstants.kIndexerMotorPort, MotorType.kBrushless);
    
    private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    private final PIDController m_IntakePID = new PIDController(0.05, 0, 0); //mess around with this later

    private final Rev2mDistanceSensor distOnboard;
    private Double angle = 0.0;
    private boolean intakeRunning = false;

    public Intake() {
        // add port
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);
        
        angle = IntakeConstants.kIntakeRaisedAngle;
    }

    /**
     * command to run the update intake function
     * <br><br>
     * placeholder since intake will have a customized default command that implements the operations controller
     */
    public Command runUpdateIntake() {
        return run(() -> this.updateIntake());
    }

    /**
     * updates intake angle with pid loops
     * <br><br>
     * called periodically from IntakeControllerCmd
     */
    public void updateIntake() {
        if (Math.abs(m_encoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.kIntakeAngleMotorEncoderOffset - angle) > 1.0) {
            moveIntake(m_IntakePID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.kIntakeAngleMotorEncoderOffset, angle) / IntakeConstants.kTurningMotorMaxSpeed);
        } else {
            stopIntakeRotation();
        }
        
        /*
        if (intakeRunning) {
            runIntake(1);

            if (distOnboard.isRangeValid() && distOnboard.getRange() < IntakeConstants.kDistanceActivationThresholdMin) {
                stopIntake();
            }
        }
        */
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
        m_IntakeAngleMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    }

    /**
     * runs intake and indexer at specified speed
     * @param speed -1 to 1, speed as a percentage of max speed
     */
    public void runIntake(double speed) {
        m_IntakeFlywheelMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0)); //don't set higher than 1, don't set lower than -1
        m_IndexerFlywheelMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    }

    /**
     * sets the angle variable to a certain value which will cause the pid loops to rotate the intake to that angle
     * @param angle radians
     */
    public void setIntakeToAngle(double angle) {
        this.angle = angle;
    }

    /**
     * start running the intake and indexer flywheel motors
     */
    public void startIntake() {
        runIntake(1);
        intakeRunning = true;
    }

    /**
     * stop the intake flywheel motors
     */
    public void stopIntake() {
        m_IntakeFlywheelMotor.stopMotor();
        intakeRunning = false;
    }

    /**
     * stop the indexer flywheel motors
     */
    public void stopIndexer() {
        m_IndexerFlywheelMotor.stopMotor();
    }

    /**
     * stop intake and indexer flywheel motors
     */
    public void stopAll() {
        stopIntake();
        stopIndexer();
    }

    /**
     * stop the intake rotation motor
     */
    public void stopIntakeRotation() {
        m_IntakeAngleMotor.stopMotor();
    }

    /**
     * lowers intake to angle set in constants
     */
    public void lowerIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeLowweredAngle);
    }
    
    /**
     * raises intake to angle set in constants
     */
    public void raiseIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngle);
    }
    
    /**
     * lowers intake to angle set in constants 
     * <br><br>
     * sets the intake and indexer flywheel motors to max speed
     */
    public void lowerAndRunIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeLowweredAngle);
        startIntake();
    }
    
    /**
     * raises intake to angle set in constants 
     * <br><br>
     * turns the intake flywheel motor off, but doesn't affect indexer flywheel motor
     */
    public void raiseAndStopIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngle);
        stopIntake();
    }
}
