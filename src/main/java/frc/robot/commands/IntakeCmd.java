package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{

    private final Intake m_intake;
    private final Trigger xButton, yButton;

    public IntakeCmd(Intake intake,
            Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton) {
        m_intake = intake;
        this.xButton = xButton;
        this.yButton = yButton;

        addRequirements(intake);
    }

    @Override
    /**
     * @return: begins the lower intake and sets the command controller to angle
     */
    public void initialize() {
        m_intake.raiseAndStopIntake();
        m_intake.stopIndexer();
    }

    @Override
    public void execute() {
        if (yButton.getAsBoolean()) {
            m_intake.stopIndexer();
            m_intake.stopIntakeRollers();
            m_intake.stopIntakeRotation();
        } else if (xButton.getAsBoolean()) {
            m_intake.lowerAndRunIntake();
        } else {
            m_intake.raiseAndStopIntake();
        }

        if (!yButton.getAsBoolean()) {
            m_intake.setAngleMotorSpeeds();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntakeRotation();
        m_intake.stopIndexer();
    }
}