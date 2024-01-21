package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax m_IntakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_IntakeRollersMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorPort, MotorType.kBrushless);
    
    private final CANcoder m_encoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderPort);

    public IntakeSubsystem() {
        
    }
}
