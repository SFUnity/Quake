package lib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.modules.SimulationSwerveModule;
import frc.robot.subsystems.modules.RealSwerveModule;

/** Interface to represent a swerve module */
public interface SwerveModule extends Sendable {

  /**
   * Creates a swerve module.
   *
   * <p>If the robot is real, a new {@link RealSwerveModule} will be created, otherwise a
   * {@link SimulationSwerveModule} will be created.
   *
   * @return A new {@link SwerveModule} based on if the robot is currently real or simulated.
   */
  public static SwerveModule create(int kDriveMotorId, int kTurningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
          int absoluteEncoderId, boolean absoluteEncoderReversed) {
    return RobotBase.isReal()
        ? new RealSwerveModule(kDriveMotorId, kTurningMotorId, driveMotorReversed, turningMotorReversed, 
                                absoluteEncoderId, absoluteEncoderReversed)
        : new SimulationSwerveModule();
  }

  // double getDrivePosition();

  // double getTurningPosition();

  // double getDriveVelocity();

  // double getTurningVelocity();

  double getAbsoluteEncoderRotations();

  /**
   * Stops the motors of the swerve module.
   */
  public default void stopMotors() {}

  /**
   * Closes the swerve module.
   * @throws Exception
   */
  public default void close() throws Exception {}

  /**
   * Returns the current state of the swerve module.
   * @return
   */
  public SwerveModuleState getState();

  /**
   * Returns the current position of the swerve module.
   * @return
   */
  public SwerveModulePosition getPosition();

  /**
   * Sets the desired state of the swerve module.
   * @param _desiredState
   */
  public void setDesiredState(SwerveModuleState _desiredState);

  /**
   * Returns the desired state of the swerve module.
   * @return
   */
  public SwerveModuleState getDesiredState();

  /**
   * Resets the encoders of the swerve module.
   */
  public void resetEncoders();

  @Override
  default void initSendable(SendableBuilder _builder) {
    // _builder.addDoubleProperty("current velocity", () -> getState().speedMetersPerSecond, null);
    // _builder.addDoubleProperty("current angle", () -> getPosition().angle.getRadians(), null);
    // _builder.addDoubleProperty("current position", () -> getPosition().distanceMeters, null);
    // _builder.addDoubleProperty("target velocity", () -> getDesiredState().speedMetersPerSecond, null);
    // _builder.addDoubleProperty("target angle", () -> getDesiredState().angle.getRadians(), null);
    // _builder.addStringProperty("Position", () -> getPosition().toString(), null);
    _builder.addStringProperty("State", () -> getState().toString(), null);
  }
}