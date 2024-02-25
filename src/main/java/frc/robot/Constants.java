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

    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackRightDriveMotorId = 7;

    public static final int kFrontLeftTurningMotorId = 2;
    public static final int kFrontRightTurningMotorId = 4;
    public static final int kBackLeftTurningMotorId = 6;
    public static final int kBackRightTurningMotorId = 8;

    // PDHId = 9
    // RoboRioId = 0

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderId = 1;
    public static final int kFrontRightDriveAbsoluteEncoderId = 0;
    public static final int kBackLeftDriveAbsoluteEncoderId = 2;
    public static final int kBackRightDriveAbsoluteEncoderId = 3;
    
    public static final int kShooterEncoderId = 4;

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
    public static final int kDriverControllerId = 0;
    public static final int kOperationControllerId = 1;

    public static final double kDeadband = 0.05;
  }

  public static final class IntakeConstants {
    public static final int kIntakeAngleMotorId = 9;
    public static final int kIntakeRollersMotorId = 10;
    public static final int kIntakeAngleMotorEncoderId = 4;

    public static final double kIntakeRollerSpeedPercent = 1.0;

    public static final double kDistanceActivationThresholdMin = 0.5; //TODO needs to be updated with measued values

    public static final double kIntakeLoweredAngleDegrees = 90; //TODO needs to be updated with measued values
    public static final double kIntakeRaisedAngleDegrees = 0;
    public static final double kIntakeAngleToleranceDegrees = 2;
    
    public static final int kIndexerMotorId = 13; //TODO needs to be updated
    public static final double kIndexerIntakeSpeedPercent = 0.5;
    public static final double kIndexerShootingSpeedPercent = 1.0;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorId = 15; //TODO needs to be updated with correct values
    public static final double kClimberHeight = 10.0; //TODO needs to be updated with correct values
    public static final double kClimberMotorMaxSpeed = 10.0; //TODO needs to be updated with correct values
  }

  public static final class ShooterConstants {
    public static final int kShooterAngleMotor = 11;
    public static final int kShooterBottomFlywheelMotorID = 5;
    public static final int kShooterTopFlywheelMotorID = 6;
    public static final int kShooterRollerMotor = 13;

    public static final int kShooterAngleMotorEncoderId = 5;
    
    public static final double kAngleToleranceDegrees = 1;
    public static final double kShooterManualAngleDegrees = 60; //TODO needs to be set
    public static final double kDesiredAmpAngleDegrees = 65;
    public static final double kSourceAngleDegrees = 60; //TODO needs to be updated with measured values
    
    public static final double kRollerIntakeSpeedPercent = 1.0; //TODO needs to be updated
    public static final double kRollerShootingSpeedPercent = 1.0; //TODO needs to be updated
    
    public static final double kFlywheelIntakeSpeedRPM = -500; //TODO needs to be updated
    public static final double kAmpShootingSpeedRPM = 4000; //TODO needs to be updated
    public static final double kShooterReadySpeedRPM = 4000; //TODO needs to be updated
    public static final double kShooterDefaultSpeedRPM = 4000; //TODO needs to be updated
    public static final double kFlywheelToleranceRPM = 10; //TODO needs to be updated

    public static final double kShooterDistanceRangeInches = 2.0; //TODO needs to be updated with measured values
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
