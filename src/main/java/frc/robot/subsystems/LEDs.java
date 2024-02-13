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
    // Default to a length of 60, start empty output
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private int m_rainbowFirstPixelHue;

    public LEDs() {
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
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

    public Command NoteInIndexerPattern() {
        return runOnce(() -> this.setRGB(LEDConstants.kNoteInIndexer[0], LEDConstants.kNoteInIndexer[1], LEDConstants.kNoteInIndexer[2]));
    }

    public Command NoteInShooterPattern() {
        return runOnce(() -> this.setRGB(LEDConstants.kNoteInShooter[0], LEDConstants.kNoteInShooter[1], LEDConstants.kNoteInShooter[2]));
    }

    public Command ShootingNotePattern() {
        return runOnce(() -> this.setRGB(LEDConstants.kShootingNote[0], LEDConstants.kShootingNote[1], LEDConstants.kShootingNote[2]));
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
