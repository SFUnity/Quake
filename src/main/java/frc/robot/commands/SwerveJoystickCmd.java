package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Trigger fieldOrientedFunction;
    private final SlewRateLimiter linearLimiter, turningLimiter;

    /**
     * @param swerveSubsystem
     * @param xSpdFunction
     * @param ySpdFunction
     * @param turningSpdFunction
     * @param fieldOrientedFunction
     */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction, Trigger fieldOrientedFunction) {
        m_swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.linearLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed = this.applyDeadBand(xSpeed);
        ySpeed = this.applyDeadBand(ySpeed);
        turningSpeed = this.applyDeadBand(turningSpeed);

        xSpeed = this.smoothLinearSpeed(xSpeed);
        ySpeed = this.smoothLinearSpeed(ySpeed);
        turningSpeed = this.smoothTurningSpeed(turningSpeed);
        
        ChassisSpeeds chassisSpeeds = speedsToChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerveSubsystem.setModuleStates(moduleStates);
    }

    public double applyDeadBand(double speed) {
        return Math.abs(speed) > OperatorConstants.kDeadband ? speed : 0.0;
    }

    public double smoothLinearSpeed(double speed) {
        return linearLimiter.calculate(speed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    }

    public double smoothTurningSpeed(double turningSpeed) {
        return turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }

    public ChassisSpeeds speedsToChassisSpeeds(double xSpeed, double ySpeed, double turningSpeed) {
        if (fieldOrientedFunction.getAsBoolean()) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_swerveSubsystem.getRotation2d());
        } else {
            return new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
