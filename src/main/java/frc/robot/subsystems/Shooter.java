package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterFlywheelMotor;
    private final CANSparkMax m_shooterRollerMotor;

    private final CANcoder m_angleEncoder;
    public final RelativeEncoder m_flywheelEncoder;
    
    private final Rev2mDistanceSensor m_shooterDistanceSensor;
    
    private final PIDController m_anglePidController;
    private final PIDController m_flywheePidController;

    public Shooter() {        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterDistanceSensor.setDistanceUnits(Unit.kInches);
        
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterFlywheelMotor, MotorType.kBrushless);
        m_shooterRollerMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);
        
        m_angleEncoder = new CANcoder(ShooterConstants.kShooterAngleMotorEncoderPort); // TODO set encoder resolution to be in degrees
        m_flywheelEncoder = m_shooterFlywheelMotor.getEncoder();

        m_anglePidController =  new PIDController(0.5,0,0);
        m_anglePidController.setTolerance(ShooterConstants.kAngleToleranceDegrees);
        m_anglePidController.setSetpoint(ShooterConstants.kShooterStartingAngle);

        m_flywheePidController = new PIDController(0.5, 0, 0);
        m_flywheePidController.setTolerance(ShooterConstants.kFlywheelToleranceRPM);
        m_flywheePidController.setSetpoint(0);
    }

    public void shootSpeaker() {
        m_flywheePidController.setSetpoint(ShooterConstants.kShooterDefaultSpeedRPM);
    }

    public void shootAmp() {
        m_flywheePidController.setSetpoint(ShooterConstants.kAmpShootingSpeedRPM);
    }

    public void readyShooter() {
        m_flywheePidController.setSetpoint(ShooterConstants.kShooterReadySpeedRPM);
    }

    public boolean shooterDoneUpdating() {
        return m_flywheePidController.atSetpoint() && m_anglePidController.atSetpoint();
    }

    /**
     * returns whether there is a note in the shooter
     * @return boolean value of if there is a note in shooter
     */
    public boolean isNoteInShooter() {
        return m_shooterDistanceSensor.isRangeValid() && m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRangeInches;
    }
    
    public void rollersIntake() {
        m_shooterRollerMotor.set(ShooterConstants.kRollerIntakeSpeedPercent);
    }

    public void rollersShooting() {
        m_shooterRollerMotor.set(1);
    }

    public void stopRollerMotors() {
        m_shooterRollerMotor.stopMotor();
    }

    public void stopFlywheelMotors() {
        m_shooterFlywheelMotor.stopMotor();
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
        double heightOfTarget = ShooterConstants.kHeightOfSpeakerInches;  // TODO MEASURE PROPER HEIGHT
        double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
        double angleDeg = Math.toDegrees(angleRad);
        return angleDeg;
    }

    /**
     * sets the setpoint of the shooter
     * @param angle angle in degrees
     */
    public void setShooterToAngle(double angle) {
        m_anglePidController.setSetpoint(angle);
    }

    public void setAngleMotorSpeeds() {
        m_shooterAngleMotor.set(m_anglePidController.calculate(m_angleEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    public void setFlywheelMotorSpeed() {
        m_shooterFlywheelMotor.set(m_flywheePidController.calculate(m_flywheelEncoder.getVelocity()));
    }
}