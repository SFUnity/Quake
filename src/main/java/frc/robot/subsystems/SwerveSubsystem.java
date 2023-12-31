package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import lib.SwerveModule;

public class SwerveSubsystem extends SubsystemBase implements AutoCloseable {
    private final SwerveModule m_frontLeft = SwerveModule.create(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_frontRight = SwerveModule.create(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backLeft = SwerveModule.create(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backRight = SwerveModule.create(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final List<SwerveModule> modules = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    private WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);
    private final BasePigeonSimCollection gyroSim = m_gyro.getSimCollection();

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, 
        new Pose2d(new Translation2d(4,4), new Rotation2d())
    );

    private final Field2d field2d = new Field2d();
    private final FieldObject2d[] modules2d = new FieldObject2d[modules.size()];

    private DoubleArrayLogEntry m_statesLog;
    private DoubleArrayLogEntry m_desiredStatesLog;

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

        for (int i = 0; i < modules2d.length; i++) {
            modules2d[i] = field2d.getObject("module-" + i);
        }

        SmartDashboard.putData("Field", field2d);

        DataLogManager.start("logs");
        DataLog log = DataLogManager.getLog();

        m_statesLog = new DoubleArrayLogEntry(log, "states");
        m_desiredStatesLog = new DoubleArrayLogEntry(log, "desired states");
    }

    // ! For testing purposes only
    public SwerveSubsystem(WPI_Pigeon2 gyro) {
        m_gyro = gyro;

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },  
            pose
        );
    }

    public void updatePoseEstimator(){
        poseEstimator.update(
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );

        field2d.setRobotPose(getPose());

        SmartDashboard.putString("Pose", getPose().toString());

        double[] states = {
            m_frontLeft.getState().angle.getDegrees(), m_frontLeft.getState().speedMetersPerSecond,
            m_frontRight.getState().angle.getDegrees(), m_frontRight.getState().speedMetersPerSecond,
            m_backLeft.getState().angle.getDegrees(), m_backLeft.getState().speedMetersPerSecond,
            m_backRight.getState().angle.getDegrees(), m_backRight.getState().speedMetersPerSecond
        };

        m_statesLog.append(states);


        for (int i = 0; i < modules.size(); i++) {
            var transform = new Transform2d(DriveConstants.kModuleOffset[i], modules.get(i).getPosition().angle);
            modules2d[i].setPose(getPose().transformBy(transform));
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);

        double[] desiredStatesLog = {
            desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
            desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
            desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
            desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond
        };

        m_desiredStatesLog.append(desiredStatesLog);
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_backLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backRight.resetEncoders();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        // Normalizes the heading to be between -180 and 180
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void simulate(){
        SmartDashboard.putData("Module 1", m_frontLeft);
        SmartDashboard.putData("Module 2", m_frontRight);
        SmartDashboard.putData("Module 3", m_backLeft);
        SmartDashboard.putData("Module 4", m_backRight);
        gyroSim.addHeading(Units.radiansToDegrees(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) * 0.02);
        SmartDashboard.putNumber("Heading", getHeading());
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        };
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_backLeft.stopMotors();
        m_backRight.stopMotors();
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
