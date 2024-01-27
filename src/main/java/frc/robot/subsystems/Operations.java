package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.OperationsConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Operations extends SubsystemBase {
    
    private final CANSparkMax m_operationsMotor;
    private final RelativeEncoder m_operationsEncoder;

    private final PIDController m_climberPID = new PIDController(0.05, 0, 0); //mess around with this later

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    private final AddressableLED m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private int m_rainbowFirstPixelHue;

    public Operations() {
        m_operationsMotor = new CANSparkMax(OperationsConstants.kIndexMotorID, MotorType.kBrushless);
        m_operationsEncoder = m_operationsMotor.getEncoder();

        
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        
    }

    public void setOperationsSpeed(double speed){
           m_operationsMotor.set(speed); 
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }

    public void setGreen() {
        setRGB(0, 255, 0);
    }
    
    public void setRGB(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(m_ledBuffer);
    }

    public Command setToRainbow() {
        return run(() -> this.rainbow());
    }


    // Climber
    public void raiseRobot() {

    }

    public void lowwerRobot() {

    }
}
