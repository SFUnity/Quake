package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{

    private final CANSparkMax m_IntakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IntakeRollersMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    
    private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    private final PIDController m_IntakePID = new PIDController(0.05, 0, 0); //mess around with this later

    private final Rev2mDistanceSensor distOnboard, distMXP;
    private Double angle = 0.0;
    private Boolean intakeMoving, intakeRunning = false;

    public Intake() {
        // add port
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distMXP = new Rev2mDistanceSensor(Port.kMXP);
        distOnboard.setAutomaticMode(true);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (intakeMoving) {
            moveIntake(m_IntakePID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.kIntakeAngleMotorEncoderOffset, angle) / IntakeConstants.kTurningMotorMaxSpeed);
            if (m_encoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.kIntakeAngleMotorEncoderOffset - angle < 1.0) {
                moveIntake(0);
                intakeMoving = false;
            }
        }

        if (intakeRunning) {
            runIntake(1);

            if (distOnboard.isRangeValid() && distOnboard.getRange() < IntakeConstants.kDistanceActivationThresholdMin) {
                stopIntake();
            }
        }
    }

    public void moveIntake(double speed) {
        m_IntakeAngleMotor.set(speed);
    }

    public void runIntake(double speed) {
        m_IntakeRollersMotor.set(speed);
    }

    public void setIntakeToAngle(double angle) {
        intakeMoving = true;
        this.angle = angle;
    }

    public void startIntake() {
        intakeRunning = true;
    }

    public void stopIntake() {
        intakeRunning = false;
    }
}
