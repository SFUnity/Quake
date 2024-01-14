package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants.DriveConstants;
import lib.SwerveModule;

public class SwerveSubsystem extends SubsystemBase implements AutoCloseable {
    private final SwerveModule m_frontLeft = SwerveModule.create(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_frontRight = SwerveModule.create(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backLeft = SwerveModule.create(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule m_backRight = SwerveModule.create(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final List<SwerveModule> modules = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    private Pigeon2 m_gyro = new Pigeon2(0);
    private final Pigeon2SimState gyroSim = new Pigeon2SimState(m_gyro);

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

    private double[] desiredModuleStates = { 0, 0, 0, 0, 0, 0, 0, 0 };
    private double[] currentStates = { 0, 0, 0, 0, 0, 0, 0, 0 };

    private final Field2d field2d = new Field2d();
    private final FieldObject2d[] modules2d = new FieldObject2d[modules.size()];

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private DoubleArrayTopic m_statesTopic = inst.getDoubleArrayTopic("module states");
    private DoubleArrayPublisher m_statesPublisher = m_statesTopic.publish();

    private DoubleArrayTopic m_desiredStatesTopic = inst.getDoubleArrayTopic("desired module states");
    private DoubleArrayPublisher m_desiredStatesPublisher = m_desiredStatesTopic.publish();

    public ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Subsystem");
    public ShuffleboardTab odometryTab = Shuffleboard.getTab("Odometry");

    private GenericEntry headingEntry = odometryTab.add("Heading", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

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

        odometryTab.add("Field", field2d);

        swerveTab.add("Front Left", m_frontLeft);
        swerveTab.add("Front Right", m_frontRight);
        swerveTab.add("Back Left", m_backLeft);
        swerveTab.add("Back Right", m_backRight);
    }

    // ! For testing purposes only
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

        currentStates[0] = m_frontLeft.getState().angle.getDegrees();
        currentStates[1] = m_frontLeft.getState().speedMetersPerSecond;
        currentStates[2] = m_frontRight.getState().angle.getDegrees();
        currentStates[3] = m_frontRight.getState().speedMetersPerSecond;
        currentStates[4] = m_backLeft.getState().angle.getDegrees();
        currentStates[5] = m_backLeft.getState().speedMetersPerSecond;
        currentStates[6] = m_backRight.getState().angle.getDegrees();
        currentStates[7] = m_backRight.getState().speedMetersPerSecond;

        m_statesPublisher.set(currentStates);

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

        for (int i = 0; i < modules.size(); i++) {
            desiredModuleStates[i] = desiredStates[i].angle.getDegrees();
            desiredModuleStates[i + 1] = desiredStates[i].speedMetersPerSecond;
        }

        m_desiredStatesPublisher.set(desiredModuleStates);
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
        gyroSim.addYaw(Units.radiansToDegrees(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) 
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0) * 0.02);
        headingEntry.setDouble(getHeading());
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
        return Rotation2d.fromDegrees(m_gyro.getAngle());
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
        m_gyro.close();
    }
}
