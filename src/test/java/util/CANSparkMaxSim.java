package util;

public class CANSparkMaxSim {

    public enum MotorType {
        kBrushless,
        kBrushed
    }

    private double speed;
    private MotorType motorType;

    public CANSparkMaxSim(int deviceID, MotorType motorType) {
        this.motorType = motorType;
    }

    public void set(double speed) {
        this.speed = speed;
    }

    public double get() {
        return speed;
    }

    public void restoreFactoryDefaults() {
        // Reset any settings you might implement to default
    }

    // Add any other methods from CANSparkMax here
}
