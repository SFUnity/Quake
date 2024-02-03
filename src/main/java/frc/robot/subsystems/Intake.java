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
    private Boolean intakeRunning = false;

    public Intake() {
        // add port
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);
        
        angle = IntakeConstants.kIntakeRaisedAngle;
    }

    public Command runUpdateIntake() {
        return run(() -> this.updateIntake());
    }

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

    public Boolean noteInIndexer() {
        return distOnboard.isRangeValid() && distOnboard.getRange() < IntakeConstants.kDistanceActivationThresholdMin;
    }

    public void moveIntake(double speed) {
        m_IntakeAngleMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    }

    public void runIntake(double speed) {
        m_IntakeFlywheelMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0)); //don't set higher than 1, don't set lower than -1
        m_IndexerFlywheelMotor.set(speed>0 ? Math.min(speed, 1.0) : Math.max(speed, -1.0));
    }

    public void setIntakeToAngle(double angle) {
        this.angle = angle;
    }

    public void startIntake() {
        runIntake(1);
        intakeRunning = true;
    }

    public void stopIntake() {
        m_IntakeFlywheelMotor.stopMotor();
        intakeRunning = false;
    }

    public void stopAll() {
        m_IntakeFlywheelMotor.stopMotor();
        m_IndexerFlywheelMotor.stopMotor();
    }

    public void stopIntakeRotation() {
        m_IntakeAngleMotor.stopMotor();
    }

    public void lowerIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeLowweredAngle);
    }
    
    public void raiseIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngle);
    }
    
    public void lowerAndRunIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeLowweredAngle);
        startIntake();
    }
    
    public void raiseAndStopIntake() {
        setIntakeToAngle(IntakeConstants.kIntakeRaisedAngle);
        stopIntake();
    }
}
