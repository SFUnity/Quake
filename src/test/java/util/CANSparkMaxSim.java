package util;

public class CANSparkMaxSim {

    // Inner enum for Motor type
    public enum MotorType {
        kBrushless,
        kBrushed
    }

    // Inner class to represent ControlType
    public enum ControlType {
        kDutyCycle,
        kVelocity,
        kPosition,
        kVoltage
    }

    private int deviceId;
    private MotorType motorType;
    private double speed;
    private double position;
    private double appliedVoltage;
    private ControlType currentControlType;
    private boolean isInverted;

    // Inner class for Encoder simulation
    private class EncoderSim {
        private double position = 0.0; // Position in rotations or units
        private double velocity = 0.0; // Velocity in RPM or units per second

        public double getPosition() {
            return position;
        }

        public void resetPosition() {
            position = 0.0;
        }

        public double getVelocity() {
            return velocity;
        }

        // Update the encoder readings based on the motor speed and gear ratio
        public void update(double speed, double gearRatio) {
            // Calculate motor velocity
            double motorVelocity = speed * 100;

            // Convert motor velocity to output shaft velocity considering gear ratio
            velocity = motorVelocity / gearRatio;

            position += velocity / 6000; // Assume updates occur every 1/100 of a minute
        }
    }

    private EncoderSim encoder;
    private double gearRatio; // How many rotations of the motor for one rotation of the output

    public CANSparkMaxSim(int deviceId, MotorType motorType, double gearRatio) {
        this.deviceId = deviceId;
        this.motorType = motorType;
        this.speed = 0.0;
        this.position = 0.0;
        this.appliedVoltage = 0.0;
        this.currentControlType = ControlType.kDutyCycle;
        this.isInverted = false;
        this.encoder = new EncoderSim();
        this.gearRatio = gearRatio;
    }

    public void set(double speed) {
        if (isInverted) {
            this.speed = -speed;
        } else {
            this.speed = speed;
        }
        encoder.update(this.speed, gearRatio);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    public void resetEncoder() {
        encoder.resetPosition();
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }

    public boolean getInverted() {
        return isInverted;
    }

    public void setControlType(ControlType controlType) {
        this.currentControlType = controlType;
    }

    public ControlType getControlType() {
        return currentControlType;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }

    public void setVoltage(double voltage) {
        this.appliedVoltage = voltage;
    }

    public double getVoltage() {
        return appliedVoltage;
    }

    @Override
    public String toString() {
        return "CANSparkMaxSim{" +
                "deviceId=" + deviceId +
                ", motorType=" + motorType +
                ", speed=" + speed +
                ", position=" + position +
                ", appliedVoltage=" + appliedVoltage +
                ", currentControlType=" + currentControlType +
                ", isInverted=" + isInverted +
                '}';
    }
}