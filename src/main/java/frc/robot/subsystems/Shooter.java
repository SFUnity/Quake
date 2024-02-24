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
    private final PIDController m_anglePidController;
    
    private final Rev2mDistanceSensor m_shooterDistanceSensor;
    private double desiredAngleDegrees;
    

    public final RelativeEncoder m_flywheelEncoder;

    public boolean shooterDoneUpdating;
    private final double kAngleToleranceDegrees = 1.0;

    public Shooter() {
        m_anglePidController =  new PIDController(0.5,0,0);
        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterFlywheelMotor, MotorType.kBrushless);
        m_shooterRollerMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);

        m_encoder = new CANcoder(ShooterConstants.kShooterAngleMotorEncoderPort); // TODO set encoder resolution to be in degrees
        m_flywheelEncoder = m_shooterFlywheelMotor.getEncoder();
        
        shooterDoneUpdating = false;
    }

   
    public void shoot() {
        setShooterMotors(1);
    }

    /**
     * returns whether there is a note in the shooter
     * @return boolean value of if there is a note in shooter
     */
    public boolean isNoteInShooter() {
        return m_shooterDistanceSensor.isRangeValid() && m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRange;
    }
    
    public void rollersIntake() {
        m_shooterRollerMotor.set(ShooterConstants.kRollerIntakeSpeed);
    }

    public void rollersShooting() {
        m_shooterRollerMotor.set(1);
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
        this.desiredAngleDegrees = angle;
    }

    public void updateShooter() {
        if (m_encoder.getAbsolutePosition().getValueAsDouble() - desiredAngleDegrees > kAngleToleranceDegrees) {
            startAngleMotors(m_anglePidController.calculate(m_encoder.getAbsolutePosition().getValueAsDouble(), desiredAngleDegrees));
        } else {
            stopAngleMotors();
            shooterDoneUpdating = true;
        }
    }

    public Command runUpdateShooter() {
        return run(() -> updateShooter());
    }
}