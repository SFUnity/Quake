import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleTest {
    static final double DELTA = 1e-2; // acceptable deviation range in sci. notation
    SwerveModule m_SwerveModule;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        m_SwerveModule = new SwerveModule(0, 0, false, false, 0, 0, false);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        m_SwerveModule.close(); // destroy our intake object
    }
}
