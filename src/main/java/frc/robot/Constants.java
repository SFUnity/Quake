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
    public static final double kPDrive = 0.05; // *
    public static final double kMaxModuleSpeedMPS = 5.05; // Change for krakens
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


    public static final double kDriveEncoderPositionConversionFactor = 6.12;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleOffset);

    public static final boolean kGyroReversed = true;

    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 0;
    public static final int kBackLeftDriveMotorId = 2;
    public static final int kBackRightDriveMotorId = 3;

    public static final int kFrontLeftTurningMotorId = 52;
    public static final int kFrontRightTurningMotorId = 54;
    public static final int kBackLeftTurningMotorId = 53;
    public static final int kBackRightTurningMotorId = 51;

    // PdhId = 9
    // RoboRioId = 0

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderId = 0;
    public static final int kFrontRightDriveAbsoluteEncoderId = 1;
    public static final int kBackLeftDriveAbsoluteEncoderId = 3;
    public static final int kBackRightDriveAbsoluteEncoderId = 2;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 13.00;
  }

  public static final class ControllerConstants {
    public static final int kDriverControllerId = 0;
    public static final int kOperationControllerId = 1;

    public static final double kDeadband = 0.1;
  }

  public static final class IntakeConstants {
    public static final int kIntakeAngleMotorId = 9;
    public static final int kIntakeRollersMotorId = 10;
    public static final int kIntakeAngleMotorEncoderId = 4;

    public static final double kIntakeRollerSpeedPercent = 1.0;

    public static final double kIntakeLoweredAngleRevRotations = 26;
    public static final double kIntakeRaisedAngleRevRotations = 1;
    
    public static final int kIndexerMotorId = 11;
    public static final double kIndexerIntakeSpeedPercent = 0.8;
  }
  
  public static final class ShooterConstants {
    public static final int kShooterAngleMotor = 8;
    public static final int kShooterBottomFlywheelMotorID = 5;
    public static final int kShooterTopFlywheelMotorID = 6;
    public static final int kShooterRollerMotor = 7;

    public static final int kShooterAngleMotorEncoderId = 5;
    
    public static final double kSpeakerManualAngleRevRotations = -12;
    public static final double kDesiredAmpAngleRevRotations = -20;
    public static final double kSourceAngleRevRotations = -12;
    public static final double kFeedingAngleRevRotations = -53;
    public static final double kIntakeAngleRevRotations = -53;
    
    public static final double kRollerShootingSpeedPercent = 1.0;
    
    public static final double kFlywheelIntakeSpeedRPM = -750;
    public static final double kAmpShootingSpeedBottomRPM = 2460;
    public static final double kAmpShootingSpeedTopRPM = 2033;
    public static final double kShooterDefaultSpeedRPM = 8000;
    public static final double kShooterFeedingSpeedRPM = 8000;
    
    public static final double kShooterDistanceRangeInches = 3.0;
  }

  public static final class LimelightConstants {
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double kLimelightMountAngleDegrees = 28.0; 

    // distance from the center of the Limelight lens to the floor
    public static final double kLimelightLensHeightInches = 26.5; 

    // distance from the target to the floor
    public static final double kHeightOfTagInches = 56.5;

    // height of speaker
    public static final double kHeightOfSpeakerInches = 79.2;

    public static int speakerTagID = 5;
    public static final double kTurnToTagTolerance = 1.0; // TODO make porportional to distance
  }
  
  public static final class ClimberConstants {
    public static final int kClimberMotorIdL = 12; //TODO needs to be updated with correct values
    public static final int kClimberMotorIdR = 13; //TODO needs to be updated with correct values
    public static final double kClimberDistanceConversionRate = 0.001; //TODO needs to be updated with correct values
    public static final double kExtendHeight = 10.0; //TODO needs to be updated with correct values
    public static final double kRetractHeight = 0.0; //TODO needs to be updated with correct values
  }
  
  public static final class LEDConstants {
    public static final int[] kShooterEmpty = {255, 0, 0};
    public static final int[] kNoteInShooter = {0, 0, 255};
    public static final int[] kTagDetected = {155, 0, 255};
    public static final int[] kAligned = {0, 255, 0};
  }

}
