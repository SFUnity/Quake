package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{

    private final Intake m_intake;
    private final Trigger circle;
    private GenericEntry intakeWorkingEntry;

    public IntakeCmd(Intake intake, Trigger circle, GenericEntry intakeWorkingEntry) {
        m_intake = intake;
        this.circle = circle;
        this.intakeWorkingEntry = intakeWorkingEntry;

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
        if (!intakeWorkingEntry.getBoolean(true)) {
            m_intake.safetyRaiseIntake();
            m_intake.stopIndexer();
            m_intake.stopIntakeRollers();
            return;
        // set indexer and intake roller speeds
        } else if (circle.getAsBoolean()) {
            m_intake.lowerAndRunIntake();
        } else {
            m_intake.raiseAndStopIntake();
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