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
    // private final CANSparkMax m_IntakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    // private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IndexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorPort, MotorType.kBrushless);
    
    // private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    // private final PIDController m_IntakePID = new PIDController(0.05, 0, 0); //mess around with this later

    private final Rev2mDistanceSensor distOnboard;

    public Intake() {
        // add port
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);
        
    }

    // /**
    //  * command to run the update intake function
    //  * placeholder since intake will have a customized default command
    //  */
    // public Command runUpdateIntake() {
    //     return run(() -> this.updateIntake());
    // }

    /**
     * updates intake angle with pid loops
     * called periodically from IntakeControllerCmd
     */
    // public void updateIntake() {
    //     moveIntake(m_IntakePID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble()));
    // }

    /**
     * returns a boolean based on if there is currently a note in the indexer or not
     */
    public boolean noteInIndexer() {
        return distOnboard.isRangeValid() && distOnboard.getRange() < IntakeConstants.kDistanceActivationThresholdMin;
    }

    // /**
    //  * rotates intake at specified speed
    //  * @param speed -1 to 1, speed as a percentage of max speed
    //  */
    // public void moveIntake(double speed) {
    //     m_IntakeAngleMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    // }

    // /**
    //  * runs intake and indexer at specified speed
    //  * @param speed -1 to 1, speed as a percentage of max speed
    //  */
    // public void runIntake(double speed) {
    //     m_IntakeMotor.set(Math.max(-1, Math.min(1, speed)));
    //     m_IndexerMotor.set(Math.max(-1, Math.min(1, speed)));
    // }

    public void runIndexer(double speed) {
        m_IndexerMotor.set(Math.max(-1, Math.min(1, speed)));
    }

    // /**
    //  * sets the angle variable to a certain value which will cause the pid loops to rotate the intake to that angle
    //  * @param angle radians
    //  */
    // public void setIntakeToAngle(double angle) {
    //     m_IntakePID.setSetpoint(angle);
    // }

    // /**
    //  * start running the intake and indexer  motors
    //  */
    // public void startIntake() {
    //     runIntake(1);
    // }

    public void startIndexer() {
        runIndexer(1);
    }

    // /**
    //  * stop the intake motors
    //  */
    // public void stopIntake() {
    //     m_IntakeMotor.stopMotor();
    // }

    /**
     * stop the indexer motors
     */
    public void stopIndexer() {
        m_IndexerMotor.stopMotor();
    }

    // /**
    //  * stop intake and indexer motors
    //  */
    // public void stopAll() {
    //     stopIntake();
    //     stopIndexer();
    // }

    // /**
    //  * stop the intake rotation motor
    //  */
    // public void stopIntakeRotation() {
    //     m_IntakeAngleMotor.stopMotor();
    // }

    // /**
    //  * lowers intake to angle set in constants
    //  */
    // public void lowerIntake() {
    //     setIntakeToAngle(IntakeConstants.kIntakeLoweredAngleRadians);
    // }
    
    // /**
    //  * raises intake to angle set in constants
    //  */
    // public void raiseIntake() {
    //     setIntakeToAngle(IntakeConstants.kIntakeRaisedAngleRadians);
    // }
    
    // /**
    //  * lowers intake to angle set in constants 
    //  * sets the intake and indexer  motors to max speed
    //  */
    // public void lowerAndRunIntake() {
    //     lowerIntake();
    //     startIntake();
    // }
    
    // /**
    //  * raises intake to angle set in constants 
    //  * turns the intake  motor off, but doesn't affect indexer  motor
    //  */
    // public void raiseAndStopIntake() {
    //     raiseIntake();
    //     stopIntake();
    // }
}
