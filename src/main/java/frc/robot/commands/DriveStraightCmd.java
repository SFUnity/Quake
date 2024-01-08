package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraightCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Double rotationDegrees, meters, speed; 
    private Double initialPosX, initialPosY;
    
    public DriveStraightCmd(Double speed, Double rotationDegrees, Double meters, SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        this.rotationDegrees = rotationDegrees;
        this.meters = meters;
        this.speed = speed;
        addRequirements(swerveSubsystem);
    }

    public DriveStraightCmd(Double rotationDegrees, Double meters, SwerveSubsystem swerveSubsystem) {
        this(rotationDegrees, 1.0, meters, swerveSubsystem);
    }

    @Override
    public void initialize() {
        initialPosX = m_swerveSubsystem.getPose().getX();
        initialPosY = m_swerveSubsystem.getPose().getY();
    }

    @Override
    public void execute() {
        SwerveModuleState[] module_states = {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(rotationDegrees)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(rotationDegrees))
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
