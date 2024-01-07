package frc.robot.commands;

import frc.robot.commands.DriveStraightCmd;
import frc.robot.commands.RotateDegreesCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CustomTestCmd extends SequentialCommandGroup{

    public CustomTestCmd(SwerveSubsystem swerveSubsystem) {
        addCommands(
            new DriveStraightCmd(0.0, 2.0, swerveSubsystem),
            new RotateDegreesCmd(180.0, true, swerveSubsystem),
            new DriveStraightCmd(0.0, 2.0, swerveSubsystem)
        );
    }



}
