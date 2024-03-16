package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {
    private final Shooter m_shooter;
    private final LimelightSubsystem m_limelight;
    private final Trigger leftBumper, rightBumper, square, circle, leftTrigger, rightTrigger;
    private GenericEntry intakeWorkingEntry;

    public ShooterCmd(Shooter shooter, LimelightSubsystem limelight, Trigger square, Trigger circle, Trigger leftBumper, Trigger rightBumper, Trigger leftTrigger, Trigger rightTrigger, GenericEntry intakeWorkingEntryEntry) {
        m_shooter = shooter;
        m_limelight = limelight;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.square = square;
        this.circle = circle;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger; 
        this.intakeWorkingEntry = intakeWorkingEntryEntry;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // Set flywheel and angle setpoints
        if (leftBumper.getAsBoolean()) {
            m_shooter.readyShootAmp();
        }

        if (rightBumper.getAsBoolean()) {
            m_shooter.readyShootSpeakerAutomatic(m_limelight.getDistance());
        }
        
        if (leftTrigger.getAsBoolean()) {
            m_shooter.readyShootFeed();
        }

        if (rightTrigger.getAsBoolean()) {
            m_shooter.readyShootSpeakerManual();
        }
        
        // Set flywheel and roller speeds
        if (square.getAsBoolean()) {
            m_shooter.intakeNote(intakeWorkingEntry.getBoolean(true));
            m_shooter.rollersIntake();
        } else if (circle.getAsBoolean()) {
            m_shooter.putNoteIntoFlywheels();
        } else {
            m_shooter.stopFlywheelMotors();
            m_shooter.stopRollerMotors();
        }

        if (!square.getAsBoolean()) {
            m_shooter.setAngleMotorSpeeds();
            m_shooter.setFlywheelMotorSpeed();
            // if (m_shooter.distanceSensorWorking()) {
            //     if (m_shooter.isNoteInShooter()) {
            //         m_shooter.setFlywheelMotorSpeed();
            //     } else {
            //         m_shooter.stopFlywheelMotors();
            //     }
            // } else {
            //     m_shooter.setFlywheelMotorSpeed();
            // }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFlywheelMotors();
        m_shooter.stopRollerMotors();
        m_shooter.stopAngleMotors();
    }
}
