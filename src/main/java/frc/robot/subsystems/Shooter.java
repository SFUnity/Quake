package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
//amogus
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.*;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;



public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterFlywheelMotor;

    private final CANcoder encoder;
    private final PIDController pidController;
    
    private final CANSparkMax shooterSparkMax;
    private final Rev2mDistanceSensor distOnboard;

    
    
    
    
    public Shooter(){
        encoder = new CANcoder(4);
        // We really don't know what these numbers mean 
        // if something breaks try changing these numbers
        pidController =  new PIDController(1,0,0);
        // We don't know what this does either, the funny guy online
        // told us to and I guess it works
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        shooterSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        m_shooterAngleMotor = new CANSparkMax(DriveConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterFlywheelMotor = new CANSparkMax(DriveConstants.kShooterFlywheelMotor, MotorType.kBrushless);

        
    }
}

