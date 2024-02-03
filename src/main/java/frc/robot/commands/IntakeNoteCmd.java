package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;

public class IntakeNoteCmd extends Command{

    private final Intake intake;
    private final Shooter shooter;
    private final Operations operations;

    public IntakeNoteCmd(Intake intake, Shooter shooter, Operations operations) {
        this.intake = intake;
        this.shooter = shooter;
        this.operations = operations;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
