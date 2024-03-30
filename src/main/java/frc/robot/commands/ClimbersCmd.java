package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climbers;

public class ClimbersCmd extends Command {
    private final Climbers m_climber;
    private final Trigger povUp, povDown;
    private boolean extending = false;
    private boolean retracting = false;

    public ClimbersCmd(Climbers climber, Trigger povUp, Trigger povDown) {
        m_climber = climber;
        this.povUp = povUp;
        this.povDown = povDown;
    }

    @Override
    public void execute() {
        if (povUp.getAsBoolean()) {
            extending = true;
            retracting = false;
        } else if (povDown.getAsBoolean()) {
            extending = false;
            retracting = true;
        } else if (m_climber.at0()) {
            extending = false;
            retracting = false;
        }

        if (extending) {
            m_climber.extend();
        } else if (retracting) {
            m_climber.retract();
        } else {
            m_climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
