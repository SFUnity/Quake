package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Ideal swerve module, useful for debugging */
public class GoalSwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  public SwerveModuleState getState() {
    return state;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  public void setDesiredState(SwerveModuleState _desiredState) {
    state = SwerveModuleState.optimize(_desiredState, state.angle);
    distance += state.speedMetersPerSecond * 0.02;
  }

  public SwerveModuleState getDesiredState() {
    return state;
  }

  public void resetEncoders() {}

}