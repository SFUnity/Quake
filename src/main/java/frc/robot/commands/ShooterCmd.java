package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {
    private final Shooter m_shooter;
    private final Trigger leftBumper, rightBumper, square, circle;
    private GenericEntry intakeWorkingEntry;

    public ShooterCmd(Shooter shooter, Trigger square, Trigger circle, Trigger leftBumper, Trigger rightBumper, GenericEntry intakeWorkingEntryEntry) {
        m_shooter = shooter;

        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.square = square;
        this.circle = circle;
        this.intakeWorkingEntry = intakeWorkingEntryEntry;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // Set flywheel and angle setpoints
        if (rightBumper.getAsBoolean()) {
            m_shooter.readyShootSpeaker();
        }

        if (leftBumper.getAsBoolean()) {
            m_shooter.readyShootAmp();
        }

        // Set flywheel and roller speeds
        if (square.getAsBoolean()) {
            m_shooter.intakeNote(intakeWorkingEntry.getBoolean(true));
            m_shooter.stopRollerMotors();
        } else if (circle.getAsBoolean()) {
            m_shooter.putNoteIntoFlywheels();
        } else {
            m_shooter.stopFlywheelMotors();
            m_shooter.stopRollerMotors();
        }

        // Set angle speeds
        if (!square.getAsBoolean()) {
            m_shooter.setAngleMotorSpeeds();
            m_shooter.setFlywheelMotorSpeed();
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
