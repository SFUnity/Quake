package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;

public class ClimberCmd extends Command {
    private final Climber m_climber;
    private final Trigger climbeTrigger, decendTrigger;
    private boolean buttonPressed = false;

    public ClimberCmd(Climber climber, Trigger climbTrigger, Trigger decendTrigger) {
        m_climber = climber;
        this.climbeTrigger = climbTrigger;
        this.decendTrigger = decendTrigger;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (climbeTrigger.getAsBoolean() && !buttonPressed) {
            m_climber.climb();
            buttonPressed = true;
        } else if (decendTrigger.getAsBoolean() && !buttonPressed) {
            m_climber.decend();
            buttonPressed = true;
        } else if (!climbeTrigger.getAsBoolean() && !decendTrigger.getAsBoolean()) {
            buttonPressed = false;
        }

        m_climber.updateClimber();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopAll();
    }

}
