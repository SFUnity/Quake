package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveJoystickCmd extends Command {
    private final Swerve m_swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Trigger goFastTrigger, goSlowTrigger;
    private Boolean goingFast = false, goingSlow = false;
    private final Boolean fieldOrientedFunction;

    private ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Subsystem");
    private GenericEntry driveSpeedEntry = swerveTab.addPersistent("Drive Normal", 1).withSize(2, 1).withPosition(0, 0).getEntry();
    private GenericEntry turnSpeedEntry = swerveTab.addPersistent("Turn Normal", 1).withSize(2, 1).withPosition(2, 0).getEntry();
    private GenericEntry driveSpeedFastEntry = swerveTab.addPersistent("Drive Fast", 1).withSize(2, 1).withPosition(0, 1).getEntry();
    private GenericEntry turnSpeedFastEntry = swerveTab.addPersistent("Turn Fast", 1).withSize(2, 1).withPosition(2, 1).getEntry();
    private GenericEntry goingFastEntry = swerveTab.add("Going Fast?", false).withSize(2, 1).withPosition(4, 1).getEntry();
    private GenericEntry driveSpeedSlowEntry = swerveTab.addPersistent("Drive Slow", 1).withSize(2, 1).withPosition(0, 2).getEntry();
    private GenericEntry turnSpeedSlowEntry = swerveTab.addPersistent("Turn Slow", 1).withSize(2, 1).withPosition(2, 2).getEntry();
    private GenericEntry goingSlowEntry = swerveTab.add("Going Slow?", false).withSize(2, 1).withPosition(4, 2).getEntry();

    /**
     * @param swerve
     * @param xSpdFunction
     * @param ySpdFunction
     * @param turningSpdFunction
     * @param fieldOrientedFunction
     */
    public SwerveJoystickCmd(Swerve swerve,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction, Trigger goFastTrigger, Trigger goSlowTrigger, 
            Boolean fieldOrientedFunction) {
        m_swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.goFastTrigger = goFastTrigger;
        this.goSlowTrigger = goSlowTrigger;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        xSpeed = this.applyDeadBand(xSpeed);
        ySpeed = this.applyDeadBand(ySpeed);
        turningSpeed = this.applyDeadBand(turningSpeed);
        
        // Modified speeds
        if (goFastTrigger.getAsBoolean()) {
            if (goingFast == true) {
                goingFast = false;
            } else {
                goingFast = true;
            }
            goingSlow = false;
        } else if (goSlowTrigger.getAsBoolean()) {
            if (goingSlow == true) {
                goingSlow = false;
            } else {
                goingSlow = true;
            }
            goingFast = false;
        }

        if (goingFast) {
            xSpeed *= driveSpeedFastEntry.getDouble(1);
            ySpeed *= driveSpeedFastEntry.getDouble(1);
            turningSpeed *= turnSpeedFastEntry.getDouble(1);
            goingFastEntry.setBoolean(true);
            goingSlowEntry.setBoolean(false);
        } else if (goingSlow) {
            xSpeed *= driveSpeedSlowEntry.getDouble(1);
            ySpeed *= driveSpeedSlowEntry.getDouble(1);
            turningSpeed *= turnSpeedSlowEntry.getDouble(1);
            goingSlowEntry.setBoolean(true);
            goingFastEntry.setBoolean(false);
        } else {
            xSpeed *= driveSpeedEntry.getDouble(1);
            ySpeed *= driveSpeedEntry.getDouble(1);
            turningSpeed *= turnSpeedEntry.getDouble(1);
            goingFastEntry.setBoolean(false);
            goingSlowEntry.setBoolean(false);
        }
        
        ChassisSpeeds chassisSpeeds = speedsToChassisSpeeds(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction);

        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setModuleStates(moduleStates);
    }

    public double applyDeadBand(double speed) {
        return Math.abs(speed) > ControllerConstants.kDeadband ? speed : 0.0;
    }

    public ChassisSpeeds speedsToChassisSpeeds(double xSpeed, double ySpeed, double turningSpeed, boolean fieldOriented) {
        if (fieldOriented) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_swerve.getRotation2d());
        } else {
            return new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
