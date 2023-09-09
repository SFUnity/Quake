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
  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public void setDesiredState(SwerveModuleState _desiredState);

  public SwerveModuleState getDesiredState();

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