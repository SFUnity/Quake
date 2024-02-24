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
    public static final double kPTurning = 0.25; // *
  }

  public static final class DriveConstants {
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(22.75);
    public static final Translation2d[] kModuleOffset = {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleOffset);

    public static final boolean kGyroReversed = true;

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 8;

    // PDHPort = 9
    // RoboRioPort = 0

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;
    
    public static final int kShooterEncoderPort = 4;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffset = 0.007;
    public static final double kBackLeftDriveAbsoluteEncoderOffset = 0.014;
    public static final double kFrontRightDriveAbsoluteEncoderOffset = 0.451;
    public static final double kBackRightDriveAbsoluteEncoderOffset = 0.245; // good
    /* */

    public static final double kPhysicalMaxSpeedMetersPerSecond = 13.00;
  }

  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperationControllerPort = 1;

    public static final double kDeadband = 0.05;
  }

  public static final class IntakeConstants {
    public static final double kTurningMotorMaxSpeed = 10.0;
    public static final double kIntakeRollerMotorMaxSpeed = 10.0;
    public static final double kIntakeAngleMotorMaxSpeed = 10.0;

    public static final int kIntakeAngleMotorPort = 9;
    public static final int kIntakeRollersMotorPort = 10;
    public static final int kIntakeAngleMotorEncoderPort = 4;

    public static final double kDistanceActivationThresholdMin = 0.5; //TODO needs to be updated with measued values
    public static final double kIntakeLoweredAngleRadians = 0.0; //TODO needs to be updated with measued values
    public static final double kIntakeRaisedAngleRadians = Math.PI/2; //TODO needs to be updated with measued values
    
    public static final double kIndexerMotorMaxSpeed = 10.0;
    public static final int kIndexerMotorPort = 13; //TODO needs to be updated
  }

  public static final class ShooterConstants {
    public static final int kShooterAngleMotor = 11;
    public static final int kShooterFlywheelMotor = 12;
    public static final int kShooterRollerMotor = 13;

    public static final int kShooterAngleMotorEncoderPort = 5;
    
    public static final double kAngleToleranceDegrees = 1;
    public static final double kShooterStartingAngle = 0.0; //TODO needs to be updated with measured values
    public static final double kShooterManualAngleDegrees = 60.0; //TODO needs to be set
    public static final double kDesiredAmpAngleDegrees = 65;
    
    public static final double kRollerIntakeSpeedPercent = 1.0; //TODO needs to be updated
    public static final double kRollerShootingSpeedPercent = 1.0; //TODO needs to be updated
    
    public static final double kAmpShootingSpeedRPM = 4000; //TODO needs to be updated
    public static final double kShooterReadySpeedRPM = 4000; //TODO needs to be updated
    public static final double kShooterDefaultSpeedRPM = 4000; //TODO needs to be updated
    public static final double kFlywheelToleranceRPM = 20; //TODO needs to be updated

    public static final double kShooterDistanceRange = 2.0; //TODO needs to be updated with measured values
    public static final int kHeightOfSpeakerInches = 78;
    public static final double kVisualDistanceInput = 6.0; //TODO needs to be updated
  }

  public static final class LEDConstants {
    public static final int[] kDefault = {255, 255, 255};
    public static final int[] kNoteInShooter = {0, 255, 0};
    public static final int[] kNoteInIndexer = {0, 0, 255};
    public static final int[] kShootingNote = {0, 0, 0};
  }

}
