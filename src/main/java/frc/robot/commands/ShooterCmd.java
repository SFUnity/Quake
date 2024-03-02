package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {
    private final Shooter m_shooter;
    private final Trigger leftBumper, rightBumper, square, triangle, cross;

    public ShooterCmd(Shooter shooter, Trigger square, Trigger triangle, Trigger cross, Trigger leftBumper, Trigger rightBumper) {
        m_shooter = shooter;

        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.square = square;
        this.triangle = triangle;
        this.cross = cross;

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
        if (triangle.getAsBoolean()) {
            m_shooter.stopFlywheelMotors();
            m_shooter.stopRollerMotors();
        } else if (cross.getAsBoolean()) {
            m_shooter.setFlywheelMotorSpeed();
            if (square.getAsBoolean()) {
                m_shooter.putNoteIntoFlywheels();
            } else {
                m_shooter.stopRollerMotors();
            }
        } else {
            m_shooter.stopFlywheelMotors();
            m_shooter.stopRollerMotors();
        }

        // Set angle speeds
        if (triangle.getAsBoolean()) {
            m_shooter.stopAngleMotors();
        } else {
            m_shooter.setAngleMotorSpeeds();
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
