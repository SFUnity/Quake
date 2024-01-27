package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;



public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterFlywheelMotor;

    private final CANcoder m_encoder;
    private final PIDController m_pidController;
    
    private final Rev2mDistanceSensor m_distOnboard;

    public Shooter(){
        m_encoder = new CANcoder(ShooterConstants.kShooterAngleMotorEncoderPort);
        // We really don't know what these numbers mean 
        // if something breaks try changing these numbers
        m_pidController =  new PIDController(1,0,0);
        // We don't know what this does either, the funny guy online
        // told us to and I guess it works
        m_distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterFlywheelMotor, MotorType.kBrushless);
    }
}

