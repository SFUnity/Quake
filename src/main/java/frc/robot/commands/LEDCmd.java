package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve; 

public class LEDCmd extends Command {
    private final Shooter m_shooter;
    private final LimelightSubsystem m_limelight;
    private final LEDs m_LEDs;

    private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private GenericEntry m_tolerancePorportionEntry = tuningTab.add("Tolerance Proportion", 100).getEntry();

    public LEDCmd(Shooter shooter, Swerve swerve, LimelightSubsystem limelightSubsystem, LEDs leds) {
        m_shooter = shooter;
        m_limelight = limelightSubsystem;
        m_LEDs = leds;

        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (DriverStation.isDisabled()) {
            m_LEDs.idlePattern();
        } else {
            if (m_shooter.isNoteInShooter()) {
                if (m_limelight.isTargetAvailable()) {
                    if (Math.abs(m_limelight.getTargetOffsetX()) < m_tolerancePorportionEntry.getDouble(100) / m_limelight.getDistance()) { // m_shooter.atDesiredAngle() && 
                        m_LEDs.alignedWithTagPattern();
                    } else {
                        m_LEDs.aprilTagDetectedPattern();
                    }
                } else {
                    m_LEDs.noteInShooterPattern();
                }
            } else {
                m_LEDs.shooterEmptyPattern();
            }
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}