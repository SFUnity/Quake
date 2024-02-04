package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Operations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LEDConstants;

public class ShooterCmd extends Command{

    private final Shooter m_shooter;
    private final Operations m_operations;
    private Boolean hasBeenFired; 
    private final Trigger xButton, yButton, aButton, bButton;

    public ShooterCmd(Shooter shooter, Operations operations, Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton) { // TODO Get input from visual
        m_shooter = shooter;
        m_operations = operations;
        hasBeenFired = false;
        this.xButton = xButton;
        this.yButton = yButton;
        this.aButton = aButton;
        this.bButton = bButton;
    }

    @Override
    public void initialize() {
        m_shooter.setShooterMotors(1); //1 should equal 100%
        m_shooter.setShooterToAngle(m_shooter.getAimAngle(ShooterConstants.kVisualDistanceInput)); // TODO add visual
    }

    @Override
    public void execute() {
        m_shooter.updateShooter();
        if(m_shooter.shooterDoneUpdating) {
            m_operations.setRGB(LEDConstants.kNoteInShooter[0], LEDConstants.kNoteInShooter[1], LEDConstants.kNoteInShooter[2]);
        }
        if(bButton.getAsBoolean()){
            hasBeenFired = true;
            m_shooter.startRollerMotors(1);
        }
    }

    @Override
    public boolean isFinished() {
        if(hasBeenFired && !m_shooter.isNoteInShooter()) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooterMotors();
        m_shooter.stopRollerMotors();
        // TODO add lights
    }
}
