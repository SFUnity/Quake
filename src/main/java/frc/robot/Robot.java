package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Swerve m_swerve;
  private Limelight m_limelight;

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
    m_limelight = m_robotContainer.getLimelight();
    DriverStation.silenceJoystickConnectionWarning(true);

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_limelight.setPipeline(0); // pipelines (4 red, 7 blue)
      System.out.println("Red pipeline");
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_limelight.setPipeline(1);
      System.out.println("Blue pipeline");
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

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_limelight.setPipeline(0);
      System.out.println("Red pipeline");
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_limelight.setPipeline(1);
      System.out.println("Blue pipeline");
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_limelight.setPipeline(0);
      System.out.println("Red pipeline");
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_limelight.setPipeline(1);
      System.out.println("Blue pipeline");
    }

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
