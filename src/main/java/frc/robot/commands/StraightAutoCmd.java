package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class StraightAutoCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;

    public StraightAutoCmd(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        SwerveModuleState[] module_states = {
                new SwerveModuleState(1.0, new Rotation2d(0, 0)),
                new SwerveModuleState(1.0, new Rotation2d(0, 0)),
                new SwerveModuleState(1.0, new Rotation2d(0, 0)),
                new SwerveModuleState(1.0, new Rotation2d(0, 0))
        };
        m_swerveSubsystem.setModuleStates(module_states);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
