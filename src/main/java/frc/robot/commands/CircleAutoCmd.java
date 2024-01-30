package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CircleAutoCmd extends Command {
    private final Swerve m_swerve;

    public CircleAutoCmd(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        SwerveModuleState[] module_states = {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(-135)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(-45))
        };
        m_swerve.setModuleStates(module_states);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
