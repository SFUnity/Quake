import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Intake;

public class IntakeTest {
    Intake m_intake;
    
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_intake = new Intake();
    }

    @AfterEach
    void shutdown() throws Exception {
        m_intake.close();
    }
}
