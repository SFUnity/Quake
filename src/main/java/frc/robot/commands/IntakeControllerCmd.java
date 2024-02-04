package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;

public class IntakeControllerCmd extends Command{

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Operations m_operations;
    private final Trigger xButton, yButton, aButton, bButton;

    public IntakeControllerCmd(Intake intake, Shooter shooter, Operations operations, 
            Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton) {
        m_intake = intake;
        m_shooter = shooter;
        m_operations = operations;
        this.xButton = xButton;
        this.yButton = yButton;
        this.aButton = aButton;
        this.bButton = bButton;
    }

    @Override
    public void initialize() { //start lower intake
        m_intake.raiseAndStopIntake();
    }

    @Override
    public void execute() {
        m_intake.updateIntake();

        if (xButton.getAsBoolean()) {
            m_intake.lowerAndRunIntake();
        } else {
            m_intake.raiseAndStopIntake();
        }

        if (yButton.getAsBoolean()) {
            m_intake.stopIndexer();
        }

        if (m_intake.noteInIndexer()) {
            m_operations.setRGB(LEDConstants.kNoteInIndexer[0], LEDConstants.kNoteInIndexer[1], LEDConstants.kNoteInIndexer[2]);
        }

        if (m_shooter.isNoteInShooter()) {
            m_intake.stopIndexer();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_intake.stopIntakeRotation();
        m_intake.stopAll();
        
    }

}