package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class DefaultIntake extends Command {

    private Intake m_intake;

    private XboxController m_controller;

    private double m_speed;

    public DefaultIntake(XboxController controller) {
        
        m_controller = controller;

        m_intake = kINTAKE;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
        
        m_speed = m_controller.getLeftY();

        m_intake.handle_intake(m_speed);

    }
    
}
