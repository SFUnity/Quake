package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraightCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Double rotationDegrees, meters; 
    private Double initialPosX, initialPosY;
    

    public DriveStraightCmd(Double rotationDegrees, Double meters, SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        this.rotationDegrees = rotationDegrees;
        this.meters = meters;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        initialPosX = m_swerveSubsystem.getPose().getX();
        initialPosY = m_swerveSubsystem.getPose().getY();
        SwerveModuleState[] module_states = {
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(rotationDegrees))
        };
        m_swerveSubsystem.setModuleStates(module_states);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return Math.sqrt(Math.pow(m_swerveSubsystem.getPose().getX() - initialPosX, 2) + Math.pow(m_swerveSubsystem.getPose().getY() - initialPosY, 2)) >= meters;
    }
}
