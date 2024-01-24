package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax m_IntakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IntakeRollersMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    
    private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    private final PIDController m_IntakePID = new PIDController(0.05, 0, 0); //mess around with this later

    private final Rev2mDistanceSensor noteSensor;

    public IntakeSubsystem() {
        // add port
        noteSensor = new Rev2mDistanceSensor();
    }

    public void moveIntake(double speed) {
        m_IntakeAngleMotor.set(speed);
    }

    public void runIntake(double speed) {
        m_IntakeRollersMotor.set(speed);
    }

    public void setIntakeToAngle(double angle) {
        moveIntake(m_IntakePID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble(), angle));
    }
}
