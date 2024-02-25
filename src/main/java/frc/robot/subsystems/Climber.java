package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase{
    private final CANSparkMax m_climberMotor;
    private final PIDController m_climberControllerPID;

    public Climber() {
        m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);
        m_climberControllerPID = new PIDController(0.5, 0, 0);
    }

    public void updateClimber() {
        
    }

    public void raiseClimber() {

    }

    public void lowerClimber() {

    }

    public void climb() {

    }

    public void decend() {

    }

    public void stopAll() {
        m_climberMotor.stopMotor();
    }

}
