package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_angleMotor; 
    private final CANSparkMax m_flywheelMotor;
    
    private final CANcoder m_angleEncoder;
    public final RelativeEncoder m_flywheelEncoder;

    private final PIDController m_anglePidController;
    
    private final Rev2mDistanceSensor m_noteSensor;
    private Boolean shooterMoving;
    private double desiredAngle;
    private Boolean angleSet;


    public Shooter(){
        m_angleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_flywheelMotor = new CANSparkMax(ShooterConstants.kShooterFlywheelMotor, MotorType.kBrushless);

        m_angleEncoder = new CANcoder(ShooterConstants.kShooterAngleMotorEncoderPort);
        m_flywheelEncoder = m_flywheelMotor.getEncoder();

        m_noteSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_noteSensor.setDistanceUnits(Unit.kInches);

        m_anglePidController =  new PIDController(0.5,0,0);
        angleSet = false;
    }

    public void shoot(){
        if(noteInShooter()){
            setShooterMotors(1);

            angleSet = false;
        }
        else{
            stopShooterMotors();
        }
    }

    public boolean noteInShooter() {
        if (m_noteSensor.isRangeValid()) {
            if (m_noteSensor.getRange() <= ShooterConstants.kShooterDistanceRange) {
                return true;
            } else{
                return false;
            }
        } else {
            System.out.println("Range is invalid");
            return false;
        }
    }

    public void stopShooterMotors() {
        m_flywheelMotor.stopMotor();
    }

    public void startAngleMotors(double speed){
        m_angleMotor.set(speed);
    }

    public void stopAngleMotors(){
        m_angleMotor.stopMotor();
    }

    public void setShooterMotors(double speed){
        m_flywheelMotor.set(speed);
    }

    /**
     * 
     * @param distanceFromTarget
     * @return retruns vertical angle to target in degrees
     */
    public double getAimAngle(int distanceFromTarget){
        double heightOfTarget = 6.5;  // feet
        double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
        double angleDeg = Math.toDegrees(angleRad);
        return angleDeg;
    }

   
    public void setShooterToAngle(double angle) {
        shooterMoving = true;
        this.desiredAngle = angle;
    }

   @Override
    public void periodic() {
        super.periodic();

        if (shooterMoving) {
            startAngleMotors(m_anglePidController.calculate(m_angleEncoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kShooterAngleMotorEncoderOffset, desiredAngle) / ShooterConstants.kShooterMotorMaxSpeed);
            if (m_angleEncoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kShooterAngleMotorEncoderOffset - desiredAngle < 1.0) {
                stopAngleMotors();
                
                shooterMoving = false;
            }
        }
    }
}