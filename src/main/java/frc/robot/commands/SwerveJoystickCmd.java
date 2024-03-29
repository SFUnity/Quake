package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;

public class SwerveJoystickCmd extends Command {
    private final Swerve m_swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
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
            Supplier<Double> turningSpdFunction, Boolean fieldOrientedFunction) {
        m_swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed = this.applyDeadBand(xSpeed);
        ySpeed = this.applyDeadBand(ySpeed);
        turningSpeed = this.applyDeadBand(turningSpeed);
        
        // Modified speeds
        xSpeed *= 0.3;
        ySpeed *= 0.3;
        turningSpeed *= -0.6;
        
        ChassisSpeeds chassisSpeeds = speedsToChassisSpeeds(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction);

        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setModuleStates(moduleStates);
    }

    public double applyDeadBand(double speed) {
        return Math.abs(speed) > OperatorConstants.kDeadband ? speed : 0.0;
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
