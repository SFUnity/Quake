package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateDegreesCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Double rotationDegrees;
    private final Boolean rotateRight;
    private Double initialRotation;
    

    public RotateDegreesCmd(Double rotationDegrees, Boolean rotateRight, SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        this.rotationDegrees = rotationDegrees;
        this.rotateRight = rotateRight;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        initialRotation = m_swerveSubsystem.getPose().getRotation().getDegrees();
        SwerveModuleState[] module_states = {
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(135 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(45 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(-135 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(-45 * (rotateRight ? 1 : -1)))
        };
        m_swerveSubsystem.setModuleStates(module_states);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(initialRotation - m_swerveSubsystem.getPose().getRotation().getDegrees()) >= rotationDegrees;
    }
}
