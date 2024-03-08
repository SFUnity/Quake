package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.Rev2mDistanceSensor.Port;

import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    
    public final Rev2mDistanceSensor m_shooterDistanceSensor;
    
    private final SparkPIDController m_anglePidController;
    private final SparkPIDController m_topFlywheePidController;
    private final SparkPIDController m_bottomFlywheePidController;

    private double desiredAngle;
    private double desiredSpeedBottom;
    private double desiredSpeedTop;

    private ShuffleboardTab speedsTab = Shuffleboard.getTab("Speeds");
    private GenericEntry ampSpeedBottomEntry = speedsTab.addPersistent("Amp Speed Bottom", ShooterConstants.kAmpShootingSpeedBottomRPM)
                                                    .withWidget(BuiltInWidgets.kNumberSlider)
                                                    .withProperties(Map.of("min", 0, "max", 5000))
                                                    .getEntry();
    private GenericEntry ampSpeedTopEntry = speedsTab.addPersistent("Amp Speed Top", ShooterConstants.kAmpShootingSpeedTopRPM)
                                                    .withWidget(BuiltInWidgets.kNumberSlider)
                                                    .withProperties(Map.of("min", 0, "max", 5000))
                                                    .getEntry();
    private GenericEntry ampAngleEntry = speedsTab.addPersistent("Amp Angle", ShooterConstants.kDesiredAmpAngleRevRotations)
                                                    .withWidget(BuiltInWidgets.kNumberSlider)
                                                    .withProperties(Map.of("min", 10, "max", 25))
                                                    .getEntry();
    // private GenericEntry speakerAngleEntry = speedsTab.addPersistent("Speaker Angle", ShooterConstants.kSpeakerManualAngleRevRotations)
    //                                                 .withWidget(BuiltInWidgets.kNumberSlider)
    //                                                 .withProperties(Map.of("min", 10, "max", 25))
    //                                                 .getEntry();
    
    private GenericEntry bottomFlywheelSpeedEntry = speedsTab.add("Bottom Speed", 0).getEntry();
    private GenericEntry topFlywheelSpeedEntry = speedsTab.add("Top Speed", 0).getEntry();
    private GenericEntry desiredSpeedBottomEntry = speedsTab.add("Desired Speed Bottom", 0).getEntry();
    private GenericEntry desiredSpeedTopEntry = speedsTab.add("Desired Speed Top", 0).getEntry();

    private ShuffleboardTab operationsTab = Shuffleboard.getTab("Operations");
    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
    private GenericEntry angleEntry = operationsTab.add("Shooter Angle", 0).getEntry();
    private GenericEntry distanceSensorEntry = operationsTab.add("Distance sensor", -2).getEntry();
    
    private GenericEntry noteInShooterEntry = driversTab.add("Note In Shooter?", false)
                                                        .withSize(5, 4)
                                                        .withPosition(5, 0)
                                                        .getEntry();

    private GenericEntry distanceSensorWorkingEntry = driversTab.addPersistent("Distance Sensor Working", true)
                                                                .withWidget(BuiltInWidgets.kToggleButton)
                                                                .withSize(3, 2)
                                                                .withPosition(2, 2)
                                                                .getEntry();

    public Shooter() {        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterDistanceSensor.setDistanceUnits(Unit.kInches);
        m_shooterDistanceSensor.setAutomaticMode(true);
        
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterBottomFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterBottomFlywheelMotorID, MotorType.kBrushless);
        m_shooterTopFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterTopFlywheelMotorID, MotorType.kBrushless);
        m_shooterRollerMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);
        
        m_angleEncoder = m_shooterAngleMotor.getEncoder();
        m_angleEncoder.setPositionConversionFactor(1/36/360); // 36:1 gear ratio and 360 degrees per rotation
        m_bottomFlywheelEncoder = m_shooterBottomFlywheelMotor.getEncoder();
        m_topFlywheelEncoder = m_shooterTopFlywheelMotor.getEncoder();

        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
        m_anglePidController = m_shooterAngleMotor.getPIDController();
        this.setAngleMotorSpeeds();

        desiredSpeedBottom = 0;
        desiredSpeedTop = 0;
        m_bottomFlywheePidController = m_shooterBottomFlywheelMotor.getPIDController();
        m_topFlywheePidController = m_shooterTopFlywheelMotor.getPIDController();
        this.setFlywheelMotorSpeed();
    }

    @Override
    public void periodic() {
        super.periodic();
        bottomFlywheelSpeedEntry.setDouble(m_bottomFlywheelEncoder.getVelocity());
        topFlywheelSpeedEntry.setDouble(m_topFlywheelEncoder.getVelocity());
        angleEntry.setDouble(m_angleEncoder.getPosition());
        distanceSensorEntry.setDouble(m_shooterDistanceSensor.GetRange());
        noteInShooterEntry.setBoolean(isNoteInShooter());
    }

    public void intakeNote(boolean intakeWorking) {
        m_bottomFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedRPM, ControlType.kVelocity);
        m_topFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedRPM, ControlType.kVelocity);
        if (intakeWorking) {
            m_anglePidController.setReference(0, ControlType.kPosition);
        } else {
            m_anglePidController.setReference(ShooterConstants.kSourceAngleRevRotations, ControlType.kPosition);
        }
    }
    
    public void readyShootSpeaker() {
        desiredSpeedBottom = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredSpeedTop = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
    }

    public void readyShootAmp() {
        desiredSpeedBottom = ampSpeedBottomEntry.getDouble(0);
        desiredSpeedTop = ampSpeedTopEntry.getDouble(0);
        desiredAngle = ampAngleEntry.getDouble(ShooterConstants.kDesiredAmpAngleRevRotations);
    }

    /**
     * returns whether there is a note in the shooter
     * @return boolean value of if there is a note in shooter
     */
    public boolean isNoteInShooter() {
        if (distanceSensorWorkingEntry.getBoolean(true)) {
            return m_shooterDistanceSensor.isRangeValid() && m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRangeInches;
        } else {
            return false;
        }
    }

    public boolean distanceSensorWorking() {
        return m_shooterDistanceSensor.isRangeValid() && distanceSensorWorkingEntry.getBoolean(true);
    }

    public void putNoteIntoFlywheels() {
        m_shooterRollerMotor.set(1);
    }

    public void feedNote() {
        m_shooterRollerMotor.set(0.15);
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
    // public double getAimAngle(Double distanceFromTarget) {
    //     double heightOfTarget = ShooterConstants.kHeightOfSpeakerInches;
    //     double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
    //     double angleDeg = Math.toDegrees(angleRad);
    //     return angleDeg;
    // }

    public void setAngleMotorSpeeds() {
        m_anglePidController.setReference(desiredAngle, ControlType.kPosition);
    }

    public void setFlywheelMotorSpeed() {
        m_bottomFlywheePidController.setReference(desiredSpeedBottom, ControlType.kVelocity);
        m_topFlywheePidController.setReference(desiredSpeedTop, ControlType.kVelocity);
        desiredSpeedBottomEntry.setDouble(desiredSpeedBottom);
        desiredSpeedTopEntry.setDouble(desiredSpeedTop);
    }

    // Auto Commands
    public Command intakeNoteCmd() {
        return run(() -> {
            intakeNote(true);
        }).withTimeout(2);
    }
    
    public Command readyShootAmpCommand() {
        return run(() -> {
            readyShootAmp();
            setFlywheelMotorSpeed();
            setAngleMotorSpeeds();
        }).withTimeout(1);
    }

    public Command readyShootSpeakerCommand() {
        return run(() -> {
            readyShootSpeaker();
            setFlywheelMotorSpeed();
            setAngleMotorSpeeds();
        }).withTimeout(1);
    }

    public Command putNoteIntoFlywheelsCommand() {
        return run(() -> {
            putNoteIntoFlywheels();
            setAngleMotorSpeeds();
            setFlywheelMotorSpeed();
        }).withTimeout(2);
    }

    public Command stopShootingCommand() {
        return runOnce(() -> {
            setAngleMotorSpeeds();
            stopFlywheelMotors();
            stopRollerMotors();
        });
    }
}