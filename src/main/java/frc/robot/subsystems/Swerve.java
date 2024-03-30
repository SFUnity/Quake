package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.modules.RealSwerveModule;
import lib.SwerveModule;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.*;


public class Swerve extends SubsystemBase implements AutoCloseable {
    private final RealSwerveModule m_frontLeft = new RealSwerveModule(
        DriveConstants.kFrontLeftDriveMotorId,
        DriveConstants.kFrontLeftTurningMotorId,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final RealSwerveModule m_frontRight = new RealSwerveModule(
        DriveConstants.kFrontRightDriveMotorId,
        DriveConstants.kFrontRightTurningMotorId,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final RealSwerveModule m_backLeft = new RealSwerveModule(
        DriveConstants.kBackLeftDriveMotorId,
        DriveConstants.kBackLeftTurningMotorId,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final RealSwerveModule m_backRight = new RealSwerveModule(
        DriveConstants.kBackRightDriveMotorId,
        DriveConstants.kBackRightTurningMotorId,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final List<SwerveModule> modules = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    private Pigeon2 m_gyro = new Pigeon2(0);
    private final Pigeon2SimState gyroSim = new Pigeon2SimState(m_gyro);

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, 
        new Pose2d(new Translation2d(2,2), new Rotation2d())
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
    public ShuffleboardTab loggingTab = Shuffleboard.getTab("Logging");
    public ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    private GenericEntry headingEntry = swerveTab.add("Heading", 0).withPosition(4, 0).withSize(2, 2).withWidget(BuiltInWidgets.kGyro).getEntry();
    
    private GenericEntry turnToTagPEntry = swerveTab.addPersistent("turnToTag P", 0.05).withPosition(0, 2).withSize(2, 1).getEntry();
    private GenericEntry turnToTagIEntry = swerveTab.addPersistent("turnToTag I", 0.0).withPosition(2, 2).withSize(1, 1).getEntry();
    private GenericEntry turnToTagDEntry = swerveTab.addPersistent("turnToTag D", 0.0).withPosition(4, 2).withSize(2, 1).getEntry();
    private GenericEntry turnToTagIZoneEntry = swerveTab.addPersistent("turnToTage I Zone", 3).withPosition(3, 2).withSize(1, 1).getEntry();
    private PIDController turnToTagPID = new PIDController(turnToTagPEntry.getDouble(0), turnToTagIEntry.getDouble(0), turnToTagDEntry.getDouble(0));

    private double pastTurnToTagPEntry = turnToTagPEntry.getDouble(0.05);
    private double pastTurnToTagIEntry = turnToTagIEntry.getDouble(0.05);
    private double pastTurnToTagDEntry = turnToTagDEntry.getDouble(0.0);
    private double pastTurnToTagIZoneEntry = turnToTagIZoneEntry.getDouble(0.0);

    private GenericEntry frontLeftDriveVoltageEntry = loggingTab.add("flDriveVoltage", 0.00).getEntry();
    private GenericEntry frontRightDriveVoltageEntry = loggingTab.add("frDriveVoltage", 0.00).getEntry();
    private GenericEntry backLeftDriveVoltageEntry = loggingTab.add("blDriveVoltage", 0.00).getEntry();
    private GenericEntry backRightDriveVoltageEntry = loggingTab.add("brDriveVoltage", 0.00).getEntry();

    private GenericEntry frontLeftDriveCurrentEntry = loggingTab.add("flDriveCurrent", 0.00).getEntry();
    private GenericEntry frontRightDriveCurrentEntry = loggingTab.add("frDriveCurrent", 0.00).getEntry();
    private GenericEntry backLeftDriveCurrentEntry = loggingTab.add("blDriveCurrent", 0.00).getEntry();
    private GenericEntry backRightDriveCurrentEntry = loggingTab.add("brDriveCurrent", 0.00).getEntry();

    private GenericEntry frontLeftTurningVoltageEntry = loggingTab.add("flTurningVoltage", 0.00).getEntry();
    private GenericEntry frontRightTurningVoltageEntry = loggingTab.add("frTurningVoltage", 0.00).getEntry();
    private GenericEntry backLeftTurningVoltageEntry = loggingTab.add("blTurningVoltage", 0.00).getEntry();
    private GenericEntry backRightTurningVoltageEntry = loggingTab.add("brTurningVoltage", 0.00).getEntry();

    private GenericEntry frontLeftTurningOutputCurrentEntry = loggingTab.add("flTurningOutputCurrent", 0.00).getEntry();
    private GenericEntry frontRightTurningOutputCurrentEntry = loggingTab.add("frTurningOutputCurrent", 0.00).getEntry();
    private GenericEntry backLeftTurningOutputCurrentEntry = loggingTab.add("blTurningOutputCurrent", 0.00).getEntry();
    private GenericEntry backRightTurningOutputCurrentEntry = loggingTab.add("brTurningOutputCurrent", 0.00).getEntry();

    public Swerve() {
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

        turnToTagPID.enableContinuousInput(-180, 180);
        turnToTagPID.setIZone(3.5);

        loggingTab.add("Field", field2d).withSize(5, 3).withPosition(0, 0);

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5, 0.0, 0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.009), // Rotation PID constants
                    ModuleConstants.kMaxModuleSpeedMPS, // Max module speed, in m/s
                    0.3, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        super.periodic();
        frontLeftDriveVoltageEntry.setDouble(m_frontLeft.getKrakenSupplyVoltage());
        frontLeftDriveCurrentEntry.setDouble(m_frontLeft.getKrakenSupplyCurrent());
        frontLeftTurningVoltageEntry.setDouble(m_frontLeft.getTurningSupplyVoltage());
        frontLeftTurningOutputCurrentEntry.setDouble(m_frontLeft.getTurningOutputCurrent());

        frontRightDriveVoltageEntry.setDouble(m_frontRight.getKrakenSupplyVoltage());
        frontRightDriveCurrentEntry.setDouble(m_frontRight.getKrakenSupplyCurrent());
        frontRightTurningVoltageEntry.setDouble(m_frontRight.getTurningSupplyVoltage());
        frontRightTurningOutputCurrentEntry.setDouble(m_frontRight.getTurningOutputCurrent());

        backLeftDriveCurrentEntry.setDouble(m_backLeft.getKrakenSupplyVoltage());
        backLeftDriveVoltageEntry.setDouble(m_backLeft.getKrakenSupplyCurrent());
        backLeftTurningVoltageEntry.setDouble(m_backLeft.getTurningSupplyVoltage());
        backLeftTurningOutputCurrentEntry.setDouble(m_backLeft.getTurningOutputCurrent());

        backRightDriveVoltageEntry.setDouble(m_backRight.getKrakenSupplyVoltage());
        backRightDriveCurrentEntry.setDouble(m_backRight.getKrakenSupplyCurrent());
        backRightTurningVoltageEntry.setDouble(m_backRight.getTurningSupplyVoltage());
        backRightTurningOutputCurrentEntry.setDouble(m_backRight.getTurningOutputCurrent());

        double currentP = turnToTagPEntry.getDouble(0.05);
        double currentI = turnToTagIEntry.getDouble(0.0);
        double currentD = turnToTagDEntry.getDouble(0.0);
        double currentIZone = turnToTagIZoneEntry.getDouble(0.0);
        
        if (pastTurnToTagPEntry != currentP) {
            turnToTagPID.setP(currentP);
            pastTurnToTagPEntry = currentP;
            System.out.println("New P: " + currentP);
        }
        if (pastTurnToTagIEntry != currentI) {
            turnToTagPID.setI(currentI);
            pastTurnToTagIEntry = currentI;
            System.out.println("New I: " + currentI);
        }
        if (pastTurnToTagDEntry != currentD) {
            turnToTagPID.setD(currentD);
            pastTurnToTagDEntry = currentD;
            System.out.println("New D: " + currentD);
        }

        if (pastTurnToTagIZoneEntry != currentIZone) {
            turnToTagPID.setIZone(currentIZone);
            pastTurnToTagIZoneEntry = currentIZone;
            System.out.println("New I Zone: " + currentIZone);
        }
    }

    public void resetHeading() {
        m_gyro.reset();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
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
            Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );

        field2d.setRobotPose(getPose());

        currentStates[0] = m_frontLeft.getState().angle.getDegrees();
        currentStates[1] = Math.abs(m_frontLeft.getState().speedMetersPerSecond);
        currentStates[2] = m_frontRight.getState().angle.getDegrees();
        currentStates[3] = Math.abs(m_frontRight.getState().speedMetersPerSecond);
        currentStates[4] = m_backLeft.getState().angle.getDegrees();
        currentStates[5] = Math.abs(m_backLeft.getState().speedMetersPerSecond);
        currentStates[6] = m_backRight.getState().angle.getDegrees();
        currentStates[7] = Math.abs(m_backRight.getState().speedMetersPerSecond);

        m_statesPublisher.set(currentStates);

        for (int i = 0; i < modules.size(); i++) {
            Transform2d transform = new Transform2d(DriveConstants.kModuleOffset[i], modules.get(i).getPosition().angle);
            modules2d[i].setPose(getPose().transformBy(transform));
        }

        headingEntry.setDouble(getHeading());
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), getRotation2d());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
        setModuleStates(moduleStates);
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
        
        desiredModuleStates[0] = desiredStates[0].angle.getDegrees();
        desiredModuleStates[1] = desiredStates[0].speedMetersPerSecond;
        desiredModuleStates[2] = desiredStates[1].angle.getDegrees();
        desiredModuleStates[3] = desiredStates[1].speedMetersPerSecond;
        desiredModuleStates[4] = desiredStates[2].angle.getDegrees();
        desiredModuleStates[5] = desiredStates[2].speedMetersPerSecond;
        desiredModuleStates[6] = desiredStates[3].angle.getDegrees();
        desiredModuleStates[7] = desiredStates[3].speedMetersPerSecond;
        
        m_desiredStatesPublisher.set(desiredModuleStates);
    }
    
    public void zeroHeading() {
        m_gyro.reset();
    }
    
    /**
     * @return heading in degrees from -180 to 180
     */
    public double getHeading() {
        // Normalizes the heading to be between -180 and 180
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }
    
    public void simulate(){
        gyroSim.addYaw(Units.radiansToDegrees(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) 
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0) * 0.02);
        headingEntry.setDouble(getHeading());
    }

    /**
     * @return the module states
     */
    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        };
    }
    
    /**
     * @return robot heading as a rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }
    
    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_backLeft.stopMotors();
        m_backRight.stopMotors();
    }

    /**
     * @param desiredAngleDegrees
     * @return turning speed in degrees
     */
    // Still a little fast
    public double turnToTagSpeed(double xOffset) {
        // if (m_limelight.alignedWithTag()) {
        //     return 0;
        // } else {
            return turnToTagPID.calculate(xOffset, 0);
        // }
    }

    /**
     * @param desiredAngleDegrees
     * @return
     */
    // Still a little fast
    public Command TurnToTag(double desiredAngleDegrees) {
        return run(
            () -> {
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, this.turnToTagSpeed(desiredAngleDegrees), getRotation2d());
                this.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
            })
        .until(() -> turnToTagPID.atSetpoint())
        .finallyDo(interrupted -> {
            this.stopModules(); 
        });
    }

    public Command SetXCommand() {
        return run(() -> setX());
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
