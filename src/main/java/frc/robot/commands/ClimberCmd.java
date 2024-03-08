package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;

public class ClimberCmd extends Command {
    private final Climber m_climber;
    private final Trigger climbTrigger;
    private boolean buttonPressed = false;

    public ClimberCmd(Climber climber, Trigger climbTrigger) {
        m_climber = climber;
        this.climbTrigger = climbTrigger;
    }

    @Override
    public void initialize() {
        //m_climber.resetEncoders();
    }

    @Override
    public void execute() {
        if (climbTrigger.getAsBoolean()) {
            m_climber.extend();
        } else {
            m_climber.retract();
        }
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
