package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.Rev2mDistanceSensor.Port;

import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterBottomFlywheelMotor;
    private final CANSparkMax m_shooterTopFlywheelMotor;
    private final CANSparkMax m_shooterRollerMotor;

    private final RelativeEncoder m_angleEncoder;
    public final RelativeEncoder m_bottomFlywheelEncoder;
    public final RelativeEncoder m_topFlywheelEncoder;
    
    private final Rev2mDistanceSensor m_shooterDistanceSensor;
    
    private final SparkPIDController m_anglePidController;
    private final SparkPIDController m_topFlywheePidController;
    private final SparkPIDController m_bottomFlywheePidController;

    private double desiredAngle;
    private double desiredSpeed;

    public Shooter() {        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterDistanceSensor.setDistanceUnits(Unit.kInches);
        
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterBottomFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterBottomFlywheelMotorID, MotorType.kBrushless);
        m_shooterTopFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterTopFlywheelMotorID, MotorType.kBrushless);
        m_shooterRollerMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);
        
        m_angleEncoder = m_shooterAngleMotor.getEncoder();
        m_angleEncoder.setPositionConversionFactor(1/36/360); // 36:1 gear ratio and 360 degrees per rotation
        m_bottomFlywheelEncoder = m_shooterBottomFlywheelMotor.getEncoder();
        m_topFlywheelEncoder = m_shooterTopFlywheelMotor.getEncoder();

        desiredAngle = ShooterConstants.kShooterManualAngleRevRotations;
        m_anglePidController = m_shooterAngleMotor.getPIDController();
        this.setAngleMotorSpeeds();

        desiredSpeed = 0;
        m_bottomFlywheePidController = m_shooterBottomFlywheelMotor.getPIDController();
        m_topFlywheePidController = m_shooterTopFlywheelMotor.getPIDController();
        this.setFlywheelMotorSpeed();
    }

    public void flywheelsIntake() {
        desiredSpeed = ShooterConstants.kFlywheelIntakeSpeedRPM;
    }
    
    public void readyShootSpeaker() {
        desiredSpeed = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredAngle = ShooterConstants.kShooterManualAngleRevRotations;
    }

    public void readyShootAmp() {
        desiredAngle = ShooterConstants.kAmpShootingSpeedRPM;
        desiredAngle = ShooterConstants.kDesiredAmpAngleRevRotations;
    }

    /**
     * returns whether there is a note in the shooter
     * @return boolean value of if there is a note in shooter
     */
    public boolean isNoteInShooter() {
        return m_shooterDistanceSensor.isRangeValid() && m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRangeInches;
    }
    
    public void rollersIntakeFromIndexer() {
        m_shooterRollerMotor.set(ShooterConstants.kRollerIntakeSpeedPercent);
    }

    public void putNoteIntoFlywheels() {
        m_shooterRollerMotor.set(1);
    }

    public void stopRollerMotors() {
        m_shooterRollerMotor.stopMotor();
    }

    public void stopFlywheelMotors() {
        m_shooterBottomFlywheelMotor.stopMotor();
        m_shooterTopFlywheelMotor.stopMotor();
    }

    public void stopAngleMotors() {
        m_shooterAngleMotor.stopMotor();
    }

    /**
     * gets angle to aim shooter
     * @param distanceFromTarget meters
     * @return retruns vertical angle to target in degrees
     */
    public double getAimAngle(Double distanceFromTarget) {
        double heightOfTarget = ShooterConstants.kHeightOfSpeakerInches;
        double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
        double angleDeg = Math.toDegrees(angleRad);
        return angleDeg;
    }

    public void setAngleMotorSpeeds() {
        m_anglePidController.setReference(desiredAngle, ControlType.kPosition);
    }

    public void setFlywheelMotorSpeed() {
        m_bottomFlywheePidController.setReference(desiredSpeed, ControlType.kVelocity);
        m_topFlywheePidController.setReference(desiredSpeed, ControlType.kVelocity, 2);
    }

    // Auto Commands
    public Command readyShootAmpCommand() {
        return runOnce(() -> readyShootAmp());
    }

    public Command readyShootSpeakerCommand() {
        return runOnce(() -> readyShootSpeaker());
    }

    public Command putNoteIntoFlywheelsCommand() {
        return runOnce(() -> putNoteIntoFlywheels());
    }
}