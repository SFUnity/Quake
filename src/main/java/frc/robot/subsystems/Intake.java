package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    private final CANSparkMax m_intakeAngleMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_indexerMotor;
    
    private final RelativeEncoder m_angleEncoder;

    private final SparkPIDController m_anglePidController;

    private ShuffleboardTab loggingTab = Shuffleboard.getTab("Logging");
    private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
    private GenericEntry angleEntry = loggingTab.add("Intake Angle", 0).getEntry();

    private GenericEntry intakePivotVoltageEntry = loggingTab.add("intakePivotVoltage", 0.00).getEntry();
    private GenericEntry intakePivotCurrentEntry = loggingTab.add("intakePivotOutputCurrent", 0.00).getEntry();
    private GenericEntry intakeRollersVoltageEntry = loggingTab.add("intakeRollersVoltage", 0.00).getEntry();
    private GenericEntry intakeRollersCurrentEntry = loggingTab.add("intakeRollersOutputCurrent", 0.00).getEntry();
    private GenericEntry indexerVoltageEntry = loggingTab.add("indexerVoltage", 0.00).getEntry();
    private GenericEntry indexerCurrentEntry = loggingTab.add("indexerOutputCurrent", 0.00).getEntry();

    private GenericEntry intakeLoweredAngleEntry = driversTab.addPersistent("Intake Lowered Angle", IntakeConstants.kIntakeLoweredAngleRevRotations)
                                                             .withSize(1, 1)
                                                             .withPosition(9, 2)
                                                             .getEntry();

    // private GenericEntry intakeClimbingAngleEntry = driversTab.addPersistent("Intake Climb", IntakeConstants.kIntakeRaisedAngleRevRotations)
    //                                                          .withSize(1, 1)
    //                                                          .withPosition(9, 0)
    //                                                          .getEntry();                                                         

    public Intake(Shooter shooter) {
        m_intakeAngleMotor = new CANSparkMax(IntakeConstants.kIntakeAngleMotorId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeRollersMotorId, MotorType.kBrushless);
        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorId, MotorType.kBrushless);

        m_intakeAngleMotor.setSecondaryCurrentLimit(80, 1);
        m_intakeMotor.setSecondaryCurrentLimit(80, 1);
        m_indexerMotor.setSecondaryCurrentLimit(80, 1);

        //m_angleEncoder = new CANcoder(IntakeConstants.kIntakeAngleMotorEncoderId);
        m_angleEncoder = m_intakeAngleMotor.getEncoder();

        m_anglePidController = m_intakeAngleMotor.getPIDController();
        m_anglePidController.setP(0.08);
    }

    @Override
    public void periodic() {
        super.periodic();
        angleEntry.setDouble(m_angleEncoder.getPosition());
        intakePivotVoltageEntry.setDouble(m_intakeAngleMotor.getBusVoltage());
        intakePivotCurrentEntry.setDouble(m_intakeAngleMotor.getAppliedOutput());
        intakeRollersVoltageEntry.setDouble(m_intakeMotor.getBusVoltage());
        intakeRollersCurrentEntry.setDouble(m_intakeMotor.getAppliedOutput());
        indexerVoltageEntry.setDouble(m_indexerMotor.getBusVoltage());
        indexerCurrentEntry.setDouble(m_indexerMotor.getAppliedOutput());
    }

    public void runIndexer() {
        m_indexerMotor.set(IntakeConstants.kIndexerIntakeSpeedPercent);
    }

    public void indexerOuttake() {
        m_indexerMotor.set(-IntakeConstants.kIndexerIntakeSpeedPercent);
    }

    /**
     * stop the indexer motors
     */
    public void stopIndexer() {
        m_indexerMotor.stopMotor();
    }
    
    /**
     * lowers intake to angle set in constants 
     * sets the intake motors to speed
     */
    public void lowerAndRunIntake() {
        m_anglePidController.setReference(intakeLoweredAngleEntry.getDouble(IntakeConstants.kIntakeLoweredAngleRevRotations), ControlType.kPosition);
        m_intakeMotor.set(IntakeConstants.kIntakeRollerSpeedPercent);
    }

    public void outtake() {
        m_intakeMotor.set(-IntakeConstants.kIntakeRollerSpeedPercent);
    }

    /**
     * stop the intake motors
     */
    public void stopIntakeRollers() {
        m_intakeMotor.stopMotor();
    }

    /**
     * stop the intake rotation motor
     */
    public void stopIntakeRotation() {
        m_intakeAngleMotor.stopMotor();
    }
    
    /**
     * raises intake to angle set in constants 
     * turns the intake  motor off, but doesn't affect indexer  motor
     */
    public void raiseAndStopIntake() {
        m_anglePidController.setReference(IntakeConstants.kIntakeRaisedAngleRevRotations, ControlType.kPosition);
        m_intakeMotor.stopMotor();
    }

    public void climb() {
        m_anglePidController.setReference(IntakeConstants.kIntakeClimbingAngleRevRotations, ControlType.kPosition);
        m_intakeMotor.stopMotor();
    }

    public void safetyRaiseIntake() {
        m_anglePidController.setReference(IntakeConstants.kIntakeRaisedAngleRevRotations, ControlType.kPosition);
        m_intakeMotor.stopMotor();
    }

    public Command noteInShooterCommand() {
        return run(() -> {
            stopIndexer();
            raiseAndStopIntake();
        });
    }

    public Command lowerAndRunIntakeCmd() {
        return run(() -> {
            lowerAndRunIntake();
            runIndexer();
        });
    }

    public Command raiseAndStopIntakeCmd() {
        return runOnce(() -> {
            raiseAndStopIntake();
            stopIndexer();
        });
    }
}
