package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCmd extends Command{

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Trigger xButton, yButton;

    public IntakeCmd(Intake intake, Shooter shooter,
            Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton) {
        m_intake = intake;
        m_shooter = shooter;
        this.xButton = xButton;
        this.yButton = yButton;

        addRequirements(intake);
    }

    // @Override
    // /**
    //  * @return: begins the lower intake and sets the command controller to angle
    //  */
    // public void initialize() { //start lower intake
    //     m_intake.raiseAndStopIntake();
    // }

    @Override
    public void execute() {
        //m_intake.updateIntake();

        if (xButton.getAsBoolean()) {
            //m_intake.lowerAndRunIntake();
            m_intake.startIndexer();
        } else {
            //m_intake.raiseAndStopIntake();
        }

        if (m_shooter.isNoteInShooter()) {
            m_intake.stopIndexer();
        }

        if (yButton.getAsBoolean()) {
            m_intake.stopIndexer();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    /** 
     * @return: ends program, stopping the intake and rotation of the command controller
    */
    public void end(boolean interrupted) {
        //super.end(interrupted);
        //m_intake.stopIntakeRotation();
        //m_intake.stopAll();
        m_intake.stopIndexer();
        
    }

}