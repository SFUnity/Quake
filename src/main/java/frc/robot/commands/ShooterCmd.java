package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class ShooterCmd extends Command{

    private final Shooter m_shooter;
    public boolean shootingSpeaker = false;
    private boolean automaticShooting = false;
    private boolean autoTogglePressed = false; // will be used later

    public boolean shootingAmp = false;
    private final Trigger aButton, bButton, xButton, yButton, autoToggle;

    public ShooterCmd(Shooter shooter, Trigger xButton, Trigger yButton, Trigger aButton, Trigger bButton, Trigger automaticToggle) { // TODO Get input from visual
        m_shooter = shooter;

        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;
        this.autoToggle = automaticToggle;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_shooter.updateShooter();

        //TODO visual distance input should not be a constant

        // if (autoToggle.getAsBoolean() && !autoTogglePressed) {
        //     automaticShooting = !automaticShooting;
        //     autoTogglePressed = true;
        // } else if (!autoToggle.getAsBoolean()) {
        //     autoTogglePressed = false;
        // }

        if (!automaticShooting && !shootingAmp) {
            m_shooter.setShooterToAngle(ShooterConstants.kShooterManualAngleDegrees);
        }

        if (bButton.getAsBoolean()) {
            shootingSpeaker = true;

            if (automaticShooting) {
                m_shooter.setShooterToAngle(ShooterConstants.kVisualDistanceInput); // TODO add visual
            }
        }

        if (aButton.getAsBoolean()) {
            shootingAmp = true;

            m_shooter.setShooterToAngle(ShooterConstants.kDesiredAmpAngleDegrees); // TODO add visual
        }

        // Set flywheel speeds
        if (shootingSpeaker && m_shooter.isNoteInShooter()) {
            m_shooter.shootSpeaker();
        } else if(shootingAmp && m_shooter.isNoteInShooter()) {
            m_shooter.shootAmp();
        } else if (m_shooter.isNoteInShooter()) {
            m_shooter.readyShooter();
        } else {
            m_shooter.stopShooterMotors();
            shootingSpeaker = false;
            shootingAmp = false;
        }

        // Set roller speeds
        if (shootingSpeaker && m_shooter.isNoteInShooter() && m_shooter.shooterDoneUpdating()) {
            m_shooter.rollersShooting();
        } else if (shootingAmp && m_shooter.isNoteInShooter() && m_shooter.shooterDoneUpdating()) {
            m_shooter.rollersShooting();
        } else if (xButton.getAsBoolean()) {
            m_shooter.rollersIntake();
        } else {
            m_shooter.stopRollerMotors();
        }
    }

    public boolean noteInShooter() {
        return m_shooter.shooterDoneUpdating() && m_shooter.isNoteInShooter() && !shootingSpeaker;
    }

    public boolean shootingSpeaker() {
        return (shootingSpeaker || shootingAmp) && m_shooter.isNoteInShooter();
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
