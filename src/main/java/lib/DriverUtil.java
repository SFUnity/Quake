package lib;

import java.util.function.Supplier;
import frc.robot.Constants.ControllerConstants;


public final class DriverUtil {
    public static double applyDeadBand(double speed) {
        return Math.abs(speed) > ControllerConstants.kDeadband ? speed : 0.0;
    }

    public static double[] getDriveSpeeds(Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
    
        xSpeed = DriverUtil.applyDeadBand(xSpeed);
        ySpeed = DriverUtil.applyDeadBand(ySpeed);
            
        // Modified speeds
        xSpeed *= 0.3;
        ySpeed *= 0.3;
    
        return new double[] {xSpeed, ySpeed};
      }
}
