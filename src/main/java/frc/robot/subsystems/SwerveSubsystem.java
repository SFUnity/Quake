package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase implements AutoCloseable {
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private Pigeon2 m_gyro = new Pigeon2(0);

    public SwerveSubsystem() {
        /* Threads are units of code. These threads call the zeroHeading method 1 sec 
        after the robot starts without interfering with the rest of the code */
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // For testing purposes only
    public SwerveSubsystem(Pigeon2 gyro) {
        m_gyro = gyro;

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        m_gyro.setYaw(0);
    }

    public double getHeading() {
        // Normalizes the heading to be between -360 and 360
        return Math.IEEEremainder(m_gyro.getYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_backLeft.stopMotors();
        m_backRight.stopMotors();
    }

    /**
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_backLeft.setState(desiredStates[2]);
        m_backRight.setState(desiredStates[3]);
    }

    @Override
    public void close() throws Exception {
        m_frontLeft.close();
        m_frontRight.close();
        m_backLeft.close();
        m_backRight.close();
        m_gyro.DestroyObject();
    }
}
