package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;



public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterFlywheelMotor;
    private final CANSparkMax m_shooterRollerMotor;

    private final CANcoder m_encoder;
    private final PIDController m_pidController;
    
    private final Rev2mDistanceSensor m_shooterDistanceSensor;
    private double desiredAngle;
    

    public final RelativeEncoder m_flywheelEncoder;

    public Boolean shooterDoneUpdating;

    public Shooter() {
        m_pidController =  new PIDController(0.5,0,0);
        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterFlywheelMotor, MotorType.kBrushless);
        m_shooterRollerMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);

        m_encoder = new CANcoder(ShooterConstants.kShooterAngleMotorEncoderPort);
        m_flywheelEncoder = m_shooterFlywheelMotor.getEncoder();
        
        shooterDoneUpdating = false;
    }

   

    public void shoot() {
        setShooterMotors(1);
    }

    public boolean isNoteInShooter() {
        
        if(m_shooterDistanceSensor.isRangeValid()) {
            return m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRange;
        } else {
            return false;
        }
    }

    public void startRollerMotors(double speed) {
        m_shooterRollerMotor.set(speed);
    }

    public void stopRollerMotors() {
        m_shooterRollerMotor.stopMotor();
    }

    public void stopShooterMotors() {
        m_shooterFlywheelMotor.stopMotor();
    }

    public void startAngleMotors(double speed) {
        m_shooterAngleMotor.set(speed);
    }

    public void stopAngleMotors() {
        m_shooterAngleMotor.stopMotor();
    }

    public void setShooterMotors(double speed) {
        m_shooterFlywheelMotor.set(speed);
        shooterDoneUpdating = false;
    }

    /**
     * 
     * @param distanceFromTarget
     * @return retruns vertical angle to target in degrees
     */
    public double getAimAngle(Double distanceFromTarget) {
        double heightOfTarget = ShooterConstants.kHeightOfSpeaker;  // TODO MEASURE PROPER HEIGHT
        double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
        double angleDeg = Math.toDegrees(angleRad);
        return angleDeg;
    }

    public void setShooterToAngle(double angle) {
        this.desiredAngle = angle;
    }

    public void updateShooter() {
        if (m_encoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kShooterAngleMotorEncoderOffset - desiredAngle > 1.0) {
            startAngleMotors(m_pidController.calculate(m_encoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kShooterAngleMotorEncoderOffset, desiredAngle) / ShooterConstants.kShooterMotorMaxSpeed);
        } else {
            stopAngleMotors();
            shooterDoneUpdating = true;
        }
    }

    public Command runUpdateShooter() {
        return run(() -> updateShooter());
    }
}