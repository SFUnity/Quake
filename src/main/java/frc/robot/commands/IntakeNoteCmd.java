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
    public void initialize() { //start lower intake
        // TODO Auto-generated method stub
        super.initialize();
        intake.startIntake();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() { //if note is in shooter, return true
        // TODO Auto-generated method stub
        return super.isFinished();
        /* if (shooter.isNoteInShooter()) {
            shooter.updateShooter();
        } */
    }

    @Override
    public void end(boolean interrupted) { //stop intake, end program, turn on lights
        // TODO Auto-generated method stub
        super.end(interrupted);
        operations.setRGB(k_noteInShooter[0], k_noteInShooter[1], k_noteInShooter[2]); //should change once k_noteInShooter is commited
        intake.raiseAndStopIntake();
        shooter.stopAngleMotors();
        shooter.stopRollerMotors();
        shooter.stopShooterMotors();
        
    }

    
}
