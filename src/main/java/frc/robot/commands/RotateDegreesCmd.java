package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateDegreesCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Double rotationDegrees, speed;
    private final Boolean rotateRight;
    private Double initialRotation;
    

    public RotateDegreesCmd(Double speed, Double rotationDegrees, Boolean rotateRight, SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        this.rotationDegrees = rotationDegrees;
        this.rotateRight = rotateRight;
        this.speed = speed;
        addRequirements(swerveSubsystem);
    }

    public RotateDegreesCmd(Double rotationDegrees, Boolean rotateRight, SwerveSubsystem swerveSubsystem) {
        this(rotationDegrees, 1.0, rotateRight, swerveSubsystem);
    }

    @Override
    public void initialize() {
        initialRotation = m_swerveSubsystem.getPose().getRotation().getDegrees();
    }

    @Override
    public void execute() {
        SwerveModuleState[] module_states = {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(135 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(45 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-135 * (rotateRight ? 1 : -1))),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-45 * (rotateRight ? 1 : -1)))
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
