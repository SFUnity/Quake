package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_shooterAngleMotor; 
    private final CANSparkMax m_shooterBottomFlywheelMotor;
    private final CANSparkMax m_shooterTopFlywheelMotor;
    private final CANSparkMax m_feederMotor;

    private final RelativeEncoder m_angleEncoder;
    public final RelativeEncoder m_bottomFlywheelEncoder;
    public final RelativeEncoder m_topFlywheelEncoder;
    public final RelativeEncoder m_feederEncoder;
    
    public final Rev2mDistanceSensor m_shooterDistanceSensor;
    
    private final SparkPIDController m_anglePidController;
    private final SparkPIDController m_topFlywheePidController;
    private final SparkPIDController m_bottomFlywheePidController;

    private double desiredAngle;
    private double desiredSpeedBottom;
    private double desiredSpeedTop;
    
    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
    private ShuffleboardTab loggingTab = Shuffleboard.getTab("Logging");
    private ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight");

    private GenericEntry bottomFlywheelVoltageEntry = loggingTab.add("bottomFlywheelVoltage", 0.00).getEntry();
    private GenericEntry bottomFlywheelCurrentEntry = loggingTab.add("bottomFlywheelOutputCurrent", 0.00).getEntry();
    private GenericEntry topFlywheelVoltageEntry = loggingTab.add("topFlywheelVoltage", 0.00).getEntry();
    private GenericEntry topFlywheelCurrentEntry = loggingTab.add("topFlywheelOutputCurrent", 0.00).getEntry();
    private GenericEntry feederVoltageEntry = loggingTab.add("feederVoltage", 0.00).getEntry();
    private GenericEntry feederCurrentEntry = loggingTab.add("feederOutputCurrent", 0.00).getEntry();
    private GenericEntry shooterPivotVoltageEntry = loggingTab.add("shooterPivotVoltage", 0.00).getEntry();
    private GenericEntry shooterPivotCurrentEntry = loggingTab.add("shooterPivotOutputCurrent", 0.00).getEntry();

    private GenericEntry angleEntry = loggingTab.add("Shooter Angle", 0).getEntry();
    private GenericEntry desiredAngleEntry = loggingTab.add("Desired Shooter Angle", 0).getEntry();
    private GenericEntry distanceSensorEntry = loggingTab.add("Distance sensor", 0).getEntry();
    
    private GenericEntry feederSpeedEntry = loggingTab.add("Feeder Speed", 0).getEntry();    
    private GenericEntry bottomFlywheelSpeedEntry = loggingTab.add("Bottom Speed", 0).getEntry();
    private GenericEntry topFlywheelSpeedEntry = loggingTab.add("Top Speed", 0).getEntry();
    private GenericEntry desiredSpeedBottomEntry = loggingTab.add("Desired Speed Bottom", 0).getEntry();
    private GenericEntry desiredSpeedTopEntry = loggingTab.add("Desired Speed Top", 0).getEntry();
    
    private GenericEntry noteInShooterEntry = driversTab.add("Note In Shooter?", false)
                                                        .withSize(5, 4)
                                                        .withPosition(5, 0)
                                                        .getEntry();

    private GenericEntry distanceSensorWorkingEntry = driversTab.addPersistent("Distance Sensor Working", false)
                                                                .withWidget(BuiltInWidgets.kToggleButton)
                                                                .withSize(3, 2)
                                                                .withPosition(2, 2)
                                                                .getEntry();
                                                                
    private GenericEntry autoAngleOffsetEntry = limelightTab.addPersistent("auto angle offset", 41).getEntry();                                                            

    public Shooter() {        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterDistanceSensor.setDistanceUnits(Unit.kInches);
        m_shooterDistanceSensor.setAutomaticMode(true);
        
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterBottomFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterBottomFlywheelMotorID, MotorType.kBrushless);
        m_shooterTopFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterTopFlywheelMotorID, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);
        
        m_angleEncoder = m_shooterAngleMotor.getEncoder();
        m_bottomFlywheelEncoder = m_shooterBottomFlywheelMotor.getEncoder();
        m_topFlywheelEncoder = m_shooterTopFlywheelMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();

        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
        m_anglePidController = m_shooterAngleMotor.getPIDController();
        m_anglePidController.setP(0.05);
        m_anglePidController.setI(0.00015);
        m_anglePidController.setIZone(3);
        this.setAngleMotorSpeeds();

        desiredSpeedBottom = 0;
        desiredSpeedTop = 0;
        m_bottomFlywheePidController = m_shooterBottomFlywheelMotor.getPIDController();
        m_topFlywheePidController = m_shooterTopFlywheelMotor.getPIDController();
        m_bottomFlywheePidController.setP(0.0002);
        m_bottomFlywheePidController.setI(0.0000001);
        m_bottomFlywheePidController.setD(0.02);
        m_topFlywheePidController.setP(0.0002);
        m_topFlywheePidController.setI(0.0000001);
        m_topFlywheePidController.setD(0.02);
        this.setFlywheelMotorSpeed();
    }

    @Override
    public void periodic() {
        super.periodic();
        bottomFlywheelSpeedEntry.setDouble(m_bottomFlywheelEncoder.getVelocity());
        topFlywheelSpeedEntry.setDouble(m_topFlywheelEncoder.getVelocity());
        feederSpeedEntry.setDouble(m_feederEncoder.getVelocity());
        angleEntry.setDouble(m_angleEncoder.getPosition());
        desiredAngleEntry.setDouble(desiredAngle);
        distanceSensorEntry.setDouble(m_shooterDistanceSensor.GetRange());
        noteInShooterEntry.setBoolean(isNoteInShooter());

        bottomFlywheelVoltageEntry.setDouble(m_shooterBottomFlywheelMotor.getBusVoltage());
        bottomFlywheelCurrentEntry.setDouble(m_shooterBottomFlywheelMotor.getOutputCurrent());
        topFlywheelVoltageEntry.setDouble(m_shooterTopFlywheelMotor.getBusVoltage());
        topFlywheelCurrentEntry.setDouble(m_shooterTopFlywheelMotor.getOutputCurrent());
        feederVoltageEntry.setDouble(m_feederMotor.getBusVoltage());
        feederCurrentEntry.setDouble(m_feederMotor.getOutputCurrent());
        shooterPivotVoltageEntry.setDouble(m_shooterAngleMotor.getBusVoltage());
        shooterPivotCurrentEntry.setDouble(m_shooterAngleMotor.getOutputCurrent());
    }

    public boolean atDesiredAngle() {
        return m_angleEncoder.getPosition() <= desiredAngle + 1 || m_angleEncoder.getPosition() >= desiredAngle - 1;
    }

    // TODO optimize after experimentation
    public void intakeNote(boolean intakeWorking) {
        if (!intakeWorking) {
            m_bottomFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedRPM, ControlType.kVelocity);
            m_topFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedRPM, ControlType.kVelocity);
        }
        m_anglePidController.setReference(intakeWorking ? ShooterConstants.kIntakeAngleRevRotations : ShooterConstants.kSourceAngleRevRotations, ControlType.kPosition);

        if (!isNoteInShooter()) {
            double speed = 0.13;
            m_feederMotor.set(intakeWorking ? speed : -speed);
        } else {
            m_feederMotor.set(0);
        }
    }
    
    public void readyShootSpeakerManual() {
        desiredSpeedBottom = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredSpeedTop = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
    }

    /**
     * @param distanceFromTarget meters
     */
    public void readyShootSpeakerAutomatic(double distanceFromTarget) {
        desiredSpeedBottom = ShooterConstants.kShooterDefaultSpeedRPM;
        desiredSpeedTop = ShooterConstants.kShooterDefaultSpeedRPM;
        
        double heightOfTarget = LimelightConstants.kHeightOfSpeakerInches;
        double angleRad = Math.atan(heightOfTarget / distanceFromTarget);
        double angleDeg = Math.toDegrees(angleRad);
        desiredAngle = angleDeg + autoAngleOffsetEntry.getDouble(41);
    }

    public void readyShootAmp() {
        desiredSpeedBottom = ShooterConstants.kAmpShootingSpeedBottomRPM;
        desiredSpeedTop = ShooterConstants.kAmpShootingSpeedTopRPM;
        desiredAngle = ShooterConstants.kDesiredAmpAngleRevRotations;
    }

    public void readyShootFeed() {
        desiredSpeedBottom = ShooterConstants.kShooterFeedingSpeedRPM;
        desiredSpeedTop = ShooterConstants.kShooterFeedingSpeedRPM;
        desiredAngle = ShooterConstants.kFeedingAngleRevRotations;
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
        m_feederMotor.set(1);
    }

    public void feedNote() {
        m_feederMotor.set(0.15);
    }

    public void stopRollerMotors() {
        m_feederMotor.stopMotor();
    }

    public void stopFlywheelMotors() {
        m_shooterBottomFlywheelMotor.stopMotor();
        m_shooterTopFlywheelMotor.stopMotor();
    }

    public void stopAngleMotors() {
        m_shooterAngleMotor.stopMotor();
    }

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

    public Command readyShootSpeakerCommand() {
        return run(() -> {
            readyShootSpeakerManual();
            setFlywheelMotorSpeed();
            setAngleMotorSpeeds();
        }).withTimeout(0.8);
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
            readyShootAmp();
            stopRollerMotors();
        });
    }
}