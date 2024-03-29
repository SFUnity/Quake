package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {
    private final Shooter m_shooter;
    private final Trigger leftBumper, rightBumper, square, circle, triangle, leftTrigger, rightTrigger;
    private GenericEntry intakeWorkingEntry;
    private boolean buttonPressedRecently = false, autoAligning = false;

    public ShooterCmd(Shooter shooter, Trigger square, Trigger circle, Trigger triangle, Trigger leftBumper, Trigger rightBumper, Trigger leftTrigger, Trigger rightTrigger, GenericEntry intakeWorkingEntryEntry) {
        m_shooter = shooter;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.square = square;
        this.circle = circle;
        this.triangle = triangle;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger; 
        this.intakeWorkingEntry = intakeWorkingEntryEntry;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // Set flywheel and angle setpoints
        if (leftBumper.getAsBoolean()) {
            autoAligning = false;
            m_shooter.readyShootAmp();
        }

        if (rightBumper.getAsBoolean() && !buttonPressedRecently) {
            buttonPressedRecently = true;
            if (autoAligning == false) {
                autoAligning = true;
            } else {
                autoAligning = false;
            }
        } else {
            buttonPressedRecently = false;
        }

        if (autoAligning) {
            m_shooter.readyShootSpeakerAutomatic();
        }
        
        if (leftTrigger.getAsBoolean()) {
            autoAligning = false;
            m_shooter.readyShootFeed();
        }

        if (rightTrigger.getAsBoolean()) {
            autoAligning = false;
            m_shooter.readyShootSpeakerManual();
        }
        
        // Set flywheel and roller speeds
        if (square.getAsBoolean()) {
            m_shooter.intakeNote(intakeWorkingEntry.getBoolean(true));
        } else if (circle.getAsBoolean()) {
            m_shooter.putNoteIntoFlywheels();
        } else if (triangle.getAsBoolean()) {
            m_shooter.outtake();
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
