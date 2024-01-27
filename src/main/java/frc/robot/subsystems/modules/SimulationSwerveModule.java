package frc.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.SwerveModule;

/** Ideal swerve module, useful for debugging */
public class SimulationSwerveModule implements SwerveModule {

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
    distance += state.speedMetersPerSecond * 0.04;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public double getAbsoluteEncoderRotations(){
    return 0.0;
  }

  @Override
  public void resetEncoders() {}

}