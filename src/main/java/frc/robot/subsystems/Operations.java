package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Operations {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    private final AddressableLED m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);


    public Operations() {
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setGreen() {
        setRGB(0, 255, 0);
    }
    
    private void setRGB(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(m_ledBuffer);
    }
}
