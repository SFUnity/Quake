package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{

    private final Intake m_intake;
    private final Trigger aButton, bButton;

    public IntakeCmd(Intake intake,
            Trigger aButton, Trigger bButton) {
        m_intake = intake;
        this.aButton = aButton;
        this.bButton = bButton;

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
        // set indexer and intake roller speeds
        if (bButton.getAsBoolean()) {
            m_intake.stopIndexer();
            m_intake.stopIntakeRollers();
        } else if (aButton.getAsBoolean()) {
            m_intake.lowerAndRunIntake();
        } else {
            m_intake.raiseAndStopIntake();
        }

        // set intake angle motor speeds
        if (!bButton.getAsBoolean()) {
            m_intake.setAngleMotorSpeeds();
        } else {
            m_intake.stopIntakeRotation();
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
        m_intake.stopIntakeRollers();
    }
}