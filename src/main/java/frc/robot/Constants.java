package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 150 / 7;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5; // *
  }

  public static final class DriveConstants {
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(22.75);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    /* */
    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
    /* */

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.25;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    /* Should tweak */
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDeadband = 0.05;
  }
}
