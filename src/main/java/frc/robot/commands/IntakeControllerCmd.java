package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDconstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;

public class IntakeControllerCmd extends Command{

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Operations m_operations;

    public IntakeControllerCmd(Intake intake, Shooter shooter, Operations operations) {
        m_intake = intake;
        m_shooter = shooter;
        m_operations = operations;
    }

    @Override
    public void initialize() { //start lower intake
        m_intake.lowerAndRunIntake();
    }

    @Override
    public void execute() {
        if (m_intake.noteInIndexer()) {
            m_operations.setRGB(LEDconstants.kNoteInIndexer[0], LEDconstants.kNoteInIndexer[1], LEDconstants.kNoteInIndexer[2]);
        }
    }

    @Override
    public boolean isFinished() { //if note is in shooter, return true
        return m_shooter.isNoteInShooter();
    }

    @Override
    public void end(boolean interrupted) { //stop intake, end program, turn on lights
        super.end(interrupted);
        m_operations.setRGB(LEDconstants.kNoteInShooter[0], LEDconstants.kNoteInShooter[1], LEDconstants.kNoteInShooter[2]);
        m_intake.raiseAndStopIntake();
        m_intake.stopAll();
        
    }

    
}