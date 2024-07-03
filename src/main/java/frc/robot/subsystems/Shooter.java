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

    private GenericEntry angleOffset = driversTab.addPersistent("Angle Offset", ShooterConstants.kSpeakerAngleOffsetRevRotations)
                                                 .withPosition(8, 1)
                                                 .withSize(1, 1)
                                                 .getEntry();
                                                 
    private GenericEntry hegihtOfSpeakerEntry = driversTab.addPersistent("Speaker Height", LimelightConstants.kHeightOfSpeakerInches)
                                                 .withPosition(9, 1)
                                                 .withSize(1, 1)
                                                 .getEntry();
                                                 
    private GenericEntry feedingAngleEntry = driversTab.addPersistent("Feeding Angle", ShooterConstants.kFeedingAngleRevRotations)
                                                       .withPosition(8, 0)
                                                       .withSize(1, 1)
                                                       .getEntry();      
                                                 
    private GenericEntry feedingSpeedEntry = driversTab.addPersistent("Feeding Speed", ShooterConstants.kShooterFeedingSpeedVoltage)
                                                       .withPosition(9, 0)
                                                       .withSize(1, 1)
                                                       .getEntry(); 

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
    private GenericEntry distanceSensorDistanceEntry = loggingTab.add("Distance Sensor Distance", 0).getEntry();
    private GenericEntry distanceSensorRangeIsValid = loggingTab.add("Dist Sensor Range is Valid", true).getEntry();
    
    private GenericEntry feederSpeedEntry = loggingTab.add("Feeder Speed", 0).getEntry(); 
    private GenericEntry feederAppliedOutputEntry = loggingTab.add("Feeder Applied Output", 0).getEntry();   
    private GenericEntry bottomFlywheelSpeedEntry = loggingTab.add("Bottom Speed", 0).getEntry();
    private GenericEntry topFlywheelSpeedEntry = loggingTab.add("Top Speed", 0).getEntry();
    private GenericEntry desiredSpeedBottomEntry = loggingTab.add("Desired Speed Bottom", 0).getEntry();
    private GenericEntry desiredSpeedTopEntry = loggingTab.add("Desired Speed Top", 0).getEntry();
    
    private GenericEntry noteInShooterEntry = driversTab.add("Note In Shooter?", false)
                                                        .withSize(1, 1)
                                                        .withPosition(8, 2)
                                                        .getEntry();                          

    private GenericEntry distanceSensorWorkingEntry = driversTab.addPersistent("Distance Sensor Working", false)
                                                                .withWidget(BuiltInWidgets.kToggleButton)
                                                                .withSize(3, 3)
                                                                .withPosition(5, 0)
                                                                .getEntry();

    private GenericEntry sourceAngleEntry = driversTab.addPersistent("Source", ShooterConstants.kSourceAngleRevRotations)
                                                                .withSize(1, 1)
                                                                .withPosition(0, 3)
                                                                .getEntry();                                                            
                                                                
    private GenericEntry feederDesiredSpeedEntry = loggingTab.add("Feeder Desired Speed", 0).getEntry();

    private final Limelight m_limelight;

    public Shooter(Limelight limelight) {        
        m_shooterDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_shooterDistanceSensor.setDistanceUnits(Unit.kInches);
        m_shooterDistanceSensor.setAutomaticMode(true);
        
        m_shooterAngleMotor = new CANSparkMax(ShooterConstants.kShooterAngleMotor, MotorType.kBrushless);
        m_shooterBottomFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterBottomFlywheelMotorID, MotorType.kBrushless);
        m_shooterTopFlywheelMotor = new CANSparkMax(ShooterConstants.kShooterTopFlywheelMotorID, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kShooterRollerMotor, MotorType.kBrushless);

        m_shooterAngleMotor.setSecondaryCurrentLimit(80, 1);
        m_shooterBottomFlywheelMotor.setSecondaryCurrentLimit(80, 1);
        m_shooterTopFlywheelMotor.setSecondaryCurrentLimit(80, 1);
        m_feederMotor.setSecondaryCurrentLimit(80, 1);

        m_shooterBottomFlywheelMotor.enableVoltageCompensation(10);
        m_shooterTopFlywheelMotor.enableVoltageCompensation(10);
        
        m_angleEncoder = m_shooterAngleMotor.getEncoder();
        m_bottomFlywheelEncoder = m_shooterBottomFlywheelMotor.getEncoder();
        m_topFlywheelEncoder = m_shooterTopFlywheelMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();

        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
        m_anglePidController = m_shooterAngleMotor.getPIDController();
        m_anglePidController.setP(0.15);
        // m_anglePidController.setI(0.00015);
        // m_anglePidController.setIZone(3);
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

        m_limelight = limelight;
    }

    @Override
    public void periodic() {
        super.periodic();
        bottomFlywheelSpeedEntry.setDouble(m_bottomFlywheelEncoder.getVelocity());
        topFlywheelSpeedEntry.setDouble(m_topFlywheelEncoder.getVelocity());
        feederSpeedEntry.setDouble(m_feederEncoder.getVelocity());
        feederAppliedOutputEntry.setDouble(m_feederMotor.getAppliedOutput());
        angleEntry.setDouble(m_angleEncoder.getPosition());
        desiredAngleEntry.setDouble(desiredAngle);
        distanceSensorDistanceEntry.setDouble(m_shooterDistanceSensor.GetRange());
        distanceSensorRangeIsValid.setBoolean(m_shooterDistanceSensor.isRangeValid());
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

    public void intakeNote(boolean intakeWorking) {
        if (!intakeWorking) {
            m_bottomFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedVoltage, ControlType.kVoltage);
            m_topFlywheePidController.setReference(ShooterConstants.kFlywheelIntakeSpeedVoltage, ControlType.kVoltage);
        }
        m_anglePidController.setReference(intakeWorking ? ShooterConstants.kIntakeAngleRevRotations : sourceAngleEntry.getDouble(ShooterConstants.kSourceAngleRevRotations), ControlType.kPosition);

        if (!isNoteInShooter() && distanceSensorWorking()) {
            m_feederMotor.set(intakeWorking ? ShooterConstants.kFeederIntakingSpeedPercent : -ShooterConstants.kFeederIntakingSpeedPercent);
            feederDesiredSpeedEntry.setDouble(ShooterConstants.kFeederIntakingSpeedPercent);
        } else {
            m_feederMotor.set(0);
            feederDesiredSpeedEntry.setDouble(0);
        }
    }
    
    public void readyShootSpeakerManual() {
        desiredSpeedBottom = ShooterConstants.kShooterDefaultSpeedVoltage;
        desiredSpeedTop = ShooterConstants.kShooterDefaultSpeedVoltage;
        desiredAngle = ShooterConstants.kSpeakerManualAngleRevRotations;
    }

    public void readyShootSpeakerAutomatic() {
        desiredSpeedBottom = ShooterConstants.kShooterDefaultSpeedVoltage;
        desiredSpeedTop = ShooterConstants.kShooterDefaultSpeedVoltage;
        
        double heightOfTarget = hegihtOfSpeakerEntry.getDouble(LimelightConstants.kHeightOfSpeakerInches);
        double angleRad = Math.atan(heightOfTarget / m_limelight.getDistance());
        double angleDeg = Math.toDegrees(angleRad);
        desiredAngle = angleDeg + angleOffset.getDouble(ShooterConstants.kSpeakerAngleOffsetRevRotations);
    }

    public void readyShootAmp() {
        desiredSpeedBottom = ShooterConstants.kAmpShootingSpeedBottomVoltage;
        desiredSpeedTop = ShooterConstants.kAmpShootingSpeedTopVoltage;
        desiredAngle = ShooterConstants.kDesiredAmpAngleRevRotations;
    }

    public void readyShootFeed() {
        desiredSpeedBottom = feedingSpeedEntry.getDouble(ShooterConstants.kShooterFeedingSpeedVoltage);
        desiredSpeedTop = feedingSpeedEntry.getDouble(ShooterConstants.kShooterFeedingSpeedVoltage);
        desiredAngle = feedingAngleEntry.getDouble(ShooterConstants.kFeedingAngleRevRotations);
    }

    public void climb() {
        desiredSpeedBottom = 0;
        desiredSpeedTop = 0;
        m_anglePidController.setReference(ShooterConstants.kClimbingAngleRevRotations, ControlType.kPosition);
    }

    /**
     * returns whether there is a note in the shooter
     * @return boolean value of if there is a note in shooter
     */
    public boolean isNoteInShooter() {
        return distanceSensorWorking() && m_shooterDistanceSensor.getRange() <= ShooterConstants.kShooterDistanceRangeInches;
    }

    public boolean distanceSensorWorking() {
        return m_shooterDistanceSensor.isRangeValid() && distanceSensorWorkingEntry.getBoolean(true);
    }

    public void putNoteIntoFlywheels() {
        m_feederMotor.set(1);
        feederDesiredSpeedEntry.setDouble(1);
    }

    public void stopRollerMotors() {
        m_feederMotor.stopMotor();
    }

    public void outtake() {
        m_feederMotor.set(-0.5);
        desiredAngle = ShooterConstants.kIntakeAngleRevRotations;
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
        m_bottomFlywheePidController.setReference(desiredSpeedBottom, ControlType.kVoltage);
        m_topFlywheePidController.setReference(desiredSpeedTop, ControlType.kVoltage);
        desiredSpeedBottomEntry.setDouble(desiredSpeedBottom);
        desiredSpeedTopEntry.setDouble(desiredSpeedTop);
    }

    public boolean atAngle() {
        return Math.abs(m_angleEncoder.getPosition() - desiredAngle) < 1;
    }

    // Auto Commands
    public Command intakeNoteCmd() {
        return run(() -> {
            intakeNote(true);
        }).until(() -> isNoteInShooter());
    }

    public Command readyAutoShoot() {
        return run(() -> {
            readyShootSpeakerAutomatic();
            setAngleMotorSpeeds();
            setFlywheelMotorSpeed();
        });
    }

    public Command autoShoot() {
        return run(() -> {
            readyShootSpeakerAutomatic();
            putNoteIntoFlywheels();
            setAngleMotorSpeeds();
            setFlywheelMotorSpeed();
        }).finallyDo(() -> {
            setAngleMotorSpeeds();
            readyShootAmp();
            stopRollerMotors();
        }).withTimeout(0.25);
    }

    public Command readyShootSpeakerCommand() {
        return run(() -> {
            readyShootSpeakerManual();
            setFlywheelMotorSpeed();
            setAngleMotorSpeeds();
        }).withTimeout(0.4);
    }

    public Command putNoteIntoFlywheelsCommand() {
        return run(() -> {
            putNoteIntoFlywheels();
            setAngleMotorSpeeds();
            setFlywheelMotorSpeed();
        }).finallyDo(() -> {
            setAngleMotorSpeeds();
            readyShootAmp();
            stopRollerMotors();
        }).until(() -> !isNoteInShooter()).withTimeout(1);
    }
}