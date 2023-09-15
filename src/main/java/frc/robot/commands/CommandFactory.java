package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandFactory {
    
    private final SwerveSubsystem m_swerveSubsystem;

    /*
     * Constructor for CommandFactory
     * 
     * The command factory is used to create commands that combine multiple subsystems.
     * 
     * @param _drivetrain The m_swerveSubsystem subsystem
     */
    public CommandFactory(SwerveSubsystem swerveSubsystem){
        m_swerveSubsystem = swerveSubsystem;
    }

    public Command AutoPath(String _pathName, PathConstraints _pathConstraints, HashMap<String, Command> _markers){

        //Split up path beteen markers
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(_pathName, _pathConstraints);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_swerveSubsystem::getPose, // Pose2d supplier
            m_swerveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.25, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            m_swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            _markers,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);

    }

}