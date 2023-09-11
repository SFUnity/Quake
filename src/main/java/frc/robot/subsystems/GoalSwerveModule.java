package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.SwerveModule;

/** Ideal swerve module, useful for debugging */
public class GoalSwerveModule implements SwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  public SwerveModuleState getState() {
    return state;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    state = SwerveModuleState.optimize(desiredState, state.angle);
    distance += state.speedMetersPerSecond * 0.02;
    SmartDashboard.putString("goal state", state.toString());
  }

  public SwerveModuleState getDesiredState() {
    return state;
  }

  public void resetEncoders() {}

}