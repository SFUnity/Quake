package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;
import lib.DriverUtil;

public class SwerveJoystickCmd extends Command {
    private final Swerve m_swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Trigger turnToSpeakerFunction;
    private final Boolean fieldOrientedFunction;

    /**
     * @param swerve
     * @param xSpdFunction
     * @param ySpdFunction
     * @param turningSpdFunction
     * @param fieldOrientedFunction
     */
    public SwerveJoystickCmd(Swerve swerve,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction, Trigger turnToSpeakerFunction, Boolean fieldOrientedFunction) {
        m_swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.turnToSpeakerFunction = turnToSpeakerFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double[] driveSpeeds = DriverUtil.getDriveSpeeds(xSpdFunction, ySpdFunction);
        double xSpeed = driveSpeeds[0];
        double ySpeed = driveSpeeds[1];
        double turningSpeed = turningSpdFunction.get();

        if (turnToSpeakerFunction.getAsBoolean()) {
            m_swerve.turnToSpeaker(0, xSpeed, ySpeed); //TODO get input from vision
        }

        turningSpeed = DriverUtil.applyDeadBand(turningSpeed);
        
        // Modified speeds
        turningSpeed *= -0.6;
        
        ChassisSpeeds chassisSpeeds = speedsToChassisSpeeds(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction);

        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setModuleStates(moduleStates);
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
