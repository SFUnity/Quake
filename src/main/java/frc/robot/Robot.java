package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Swerve m_swerve;
  private LEDs m_LEDs;

  // Git info logging
  StringLogEntry entryGitSha = new StringLogEntry(DataLogManager.getLog(), "/Metadata/GitSHA");
  StringLogEntry entryGitBranch = new StringLogEntry(DataLogManager.getLog(), "/Metadata/GitBranch");

  @Override
  public void robotInit() {
    // Logging stuff
    if (RobotBase.isReal()) {
      DataLogManager.start("/home/lvuser");
    } else {
      DataLogManager.start("logs");
    }
    DataLog log = DataLogManager.getLog();
    DriverStation.startDataLog(log);

    // Git info logging. Run build if it says GitBuildConstants doesn't exist
    entryGitSha.append(GitBuildConstants.GIT_SHA);
    entryGitBranch.append(GitBuildConstants.GIT_BRANCH);

    m_robotContainer = new RobotContainer();
    m_swerve = m_robotContainer.getSwerve();
    m_LEDs = m_robotContainer.getLEDs();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_LEDs.setRGB(LEDConstants.kDefault[0], LEDConstants.kDefault[1], LEDConstants.kDefault[2]);

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      LimelightConstants.speakerTagID = 4;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightConstants.speakerTagID = 7;
    } else {
      LimelightConstants.speakerTagID = 5;
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_swerve.updatePoseEstimator();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    m_swerve.simulate();
  }
}
