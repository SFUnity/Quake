package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climbers;

public class ClimbersCmd extends Command {
    private final Climbers m_climber;
    private final Trigger povUp;

    public ClimbersCmd(Climbers climber, Trigger povUp) {
        m_climber = climber;
        this.povUp = povUp;

        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        if (povUp.getAsBoolean()) {
            m_climber.extend();
        } else {
            m_climber.retract();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
