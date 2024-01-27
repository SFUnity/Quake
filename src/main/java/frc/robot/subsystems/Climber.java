package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    private final PIDController m_IntakePID = new PIDController(0.05, 0, 0); //mess around with this later

    public Climber() {

    }

    @Override
    public void periodic() {
        super.periodic();

    }

    public void raiseRobot() {

    }

    public void lowwerRobot() {

    }
}
