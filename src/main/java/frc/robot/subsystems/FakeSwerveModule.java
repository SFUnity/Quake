package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.SwerveModule;

/** Ideal swerve module, useful for debugging */
public class FakeSwerveModule implements SwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  @Override
  public SwerveModuleState getState() {
    return state;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    state = SwerveModuleState.optimize(desiredState, state.angle);
    distance += state.speedMetersPerSecond * 0.02;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public void resetEncoders() {}

}