package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    private final AddressableLED m_led = new AddressableLED(9);
    // Reuse buffer
    private final int LED_LENGTH = 150;
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    private int m_rainbowFirstPixelHue;

    private int dashStart = 0;

    public LEDs() {
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setGreen() {
        setSolid(0, 255, 0);
    }
    
    public void setSolid(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(m_ledBuffer);
    }

    public void noteInShooterPattern() {
        setSolid(LEDConstants.kNoteInShooter[0], LEDConstants.kNoteInShooter[1], LEDConstants.kNoteInShooter[2]);
    }

    public void shooterEmptyPattern() {
        setSolid(LEDConstants.kShooterEmpty[0], LEDConstants.kShooterEmpty[1], LEDConstants.kShooterEmpty[2]);
    }

    public void aprilTagDetectedPattern() {
        setSolid(LEDConstants.kTagDetected[0], LEDConstants.kTagDetected[1], LEDConstants.kTagDetected[2]);
    }

    public void alignedWithTagPattern() {
        setSolid(LEDConstants.kAligned[0], LEDConstants.kAligned[1], LEDConstants.kAligned[2]);
    }

    public void sourceIntake() {
        setSolid(LEDConstants.kSourceIntake[0], LEDConstants.kSourceIntake[1], LEDConstants.kSourceIntake[2]);
    }

    public void idlePattern() {
        int dashLength = 10;
        setSolid(255, 0, 0);

        for (int i = dashStart; i < dashStart + dashLength; i++) {
            m_ledBuffer.setRGB(i % LED_LENGTH, 255, 120, 0);
        }

        // for (int i = dashStart + 10; i < dashStart + dashLength; i++) {
        //     m_ledBuffer.setRGB(i % LED_LENGTH, 255, 120, 0);
        // }

        dashStart += LED_LENGTH * 0.02;
        dashStart %= LED_LENGTH;
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

    public Command setToRainbow() {
        return run(() -> this.rainbow());
    }
}
