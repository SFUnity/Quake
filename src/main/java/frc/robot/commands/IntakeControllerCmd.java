package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDconstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;

public class IntakeControllerCmd extends Command{

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Operations m_operations;
    private final Supplier<Boolean> xButton, yButton, aButton, bButton;

    public IntakeControllerCmd(Intake intake, Shooter shooter, Operations operations, 
            Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier<Boolean> aButton, Supplier<Boolean> bButton) {
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
        m_intake.lowerAndRunIntake();
    }

    @Override
    public void execute() {
        m_intake.updateIntake();

        if (xButton.get()) {
            m_intake.lowerAndRunIntake();
        } else {
            m_intake.raiseAndStopIntake();
        }

        if (m_intake.noteInIndexer()) {
            m_operations.setRGB(LEDconstants.kNoteInIndexer[0], LEDconstants.kNoteInIndexer[1], LEDconstants.kNoteInIndexer[2]);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_intake.raiseAndStopIntake();
        m_intake.stopAll();
        
    }

}