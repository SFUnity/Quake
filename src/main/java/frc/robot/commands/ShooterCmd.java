package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;

public class ShooterCmd extends Command{

    private final Shooter m_shooter;
    private final Operations m_operations;

    public ShooterCmd(Shooter shooter, Operations operations) { // TODO Get input from visual
        m_shooter = shooter;
        m_operations = operations;
    }

    @Override
    public void initialize() {
        m_shooter.setShooterMotors(1); //1 should equal 100%
        m_shooter.setShooterToAngle(m_shooter.getAimAngle(0)); // TODO add visual
    }

    @Override
    public void execute() {
        m_shooter.updateShooter();
        if(m_shooter.shooterDoneUpdating) {
            m_shooter.startRollerMotors(1);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO add something to say its done
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooterMotors();
        m_shooter.stopRollerMotors();
        // TODO add lights
    }
}
