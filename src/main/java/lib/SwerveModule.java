package lib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.GoalSwerveModule;
import frc.robot.subsystems.MAXSwerveModule;

/** Interface to represent a swerve module */
public interface SwerveModule extends Sendable {

  /**
   * Creates a swerve module.
   *
   * <p>If the robot is real, a new {@link MAXSwerveModule} will be created, otherwise a
   * {@link GoalSwerveModule} will be created.
   *
   * <p>For the parameters, see {@link MAXSwerveModule#MAXSwerveModule(int, int, double)}.
   *
   * @return A new {@link SwerveModule} based on if the robot is currently real or simulated.
   */
  public static SwerveModule create(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
          int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    return RobotBase.isReal()
        ? new MAXSwerveModule(kDriveMotorId, kTurningMotorId, driveMotorReversed, turningMotorReversed, 
                                absoluteEncoderId, absoluteEncoderOffset, absoluteEncoderReversed)
        : new GoalSwerveModule();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState();

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition();

  /**
   * Sets the desired state for the module.
   *
   * @param _desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState _desiredState);

  /**
   * Returns the desired state for the module.
   *
   * @return The desired state of the module.
   */
  public SwerveModuleState getDesiredState();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();

  @Override
  default void initSendable(SendableBuilder _builder) {
    _builder.addDoubleProperty("current velocity", () -> getState().speedMetersPerSecond, null);
    _builder.addDoubleProperty("current angle", () -> getPosition().angle.getRadians(), null);
    _builder.addDoubleProperty("current position", () -> getPosition().distanceMeters, null);
    _builder.addDoubleProperty("target velocity", () -> getDesiredState().speedMetersPerSecond, null);
    _builder.addDoubleProperty("target angle", () -> getDesiredState().angle.getRadians(), null);
  }
}