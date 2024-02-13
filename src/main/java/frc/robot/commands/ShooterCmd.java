package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LEDConstants;

public class ShooterCmd extends Command{

    private final Shooter m_shooter;
    private final LEDs m_LEDs;
    private boolean shootingNote = false;
    private boolean automaticShooting = true;

    private boolean shootingAmp = false;
    private final Trigger aButton, bButton;

    public ShooterCmd(Shooter shooter, LEDs operations, Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton) { // TODO Get input from visual
        m_shooter = shooter;
        m_LEDs = operations;

        this.aButton = aButton;
        this.bButton = bButton;

        addRequirements(shooter, operations);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_shooter.updateShooter();

        //TODO visual distance input should not be a constant
        

        if(m_shooter.shooterDoneUpdating && m_shooter.isNoteInShooter() && !shootingNote) {
            m_LEDs.setRGB(LEDConstants.kNoteInShooter[0], LEDConstants.kNoteInShooter[1], LEDConstants.kNoteInShooter[2]);
        }

        if (!automaticShooting && !shootingAmp) {
            m_shooter.setShooterToAngle(ShooterConstants.kShooterManualAngle);;
        }

        if (bButton.getAsBoolean()) {
            shootingNote = true;

            if (automaticShooting) {
                m_shooter.setShooterToAngle(m_shooter.getAimAngle(ShooterConstants.kVisualDistanceInput)); // TODO add visual
            }
        }

        /*
        // TODO updated with keybind
        // toggles automatic shooting
        if (false) {
            automaticShooting = !automaticShooting;
        }
        */

        if (aButton.getAsBoolean()) {
            shootingAmp = true;

            m_shooter.setShooterToAngle(m_shooter.getAimAngle(ShooterConstants.kDesiredAmpAngle)); // TODO add visual
        }

        if (shootingNote && m_shooter.isNoteInShooter()) {
            m_shooter.setShooterMotors(1); //1 should equal 100%
            m_shooter.startRollerMotors(1);
            m_LEDs.setRGB(0, 0, 0);
        } else if(shootingAmp && m_shooter.isNoteInShooter()) {
            m_shooter.setShooterMotors(ShooterConstants.kAmpShootingSpeed); //TODO should equal to some percentage
            m_shooter.startRollerMotors(1);
            m_LEDs.setRGB(0, 0, 0);
        } else if (m_shooter.isNoteInShooter()) {
            m_shooter.setShooterMotors(ShooterConstants.kShooterReadySpeed);
        } else {
            m_shooter.stopShooterMotors();
            m_shooter.stopRollerMotors();
            shootingNote = false;
            shootingAmp = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooterMotors();
        m_shooter.stopRollerMotors();
        m_shooter.stopAngleMotors();
    }
}
