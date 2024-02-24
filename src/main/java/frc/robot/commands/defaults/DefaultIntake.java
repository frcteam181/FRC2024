package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class DefaultIntake extends Command {

    private Intake m_intake;

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private double m_speed;

    private boolean m_isPS4;

    public DefaultIntake(XboxController controllerXbpox, PS4Controller controllerPS4, boolean isPS4) {

        m_isPS4 = isPS4;
        
        m_controllerXbox = controllerXbpox;
        m_controllerPS4 = controllerPS4;

        m_intake = kINTAKE;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
        
        if(m_isPS4) {
            m_speed = m_controllerPS4.getLeftY();
        } else {
            m_speed = m_controllerXbox.getLeftY();
        }
        m_intake.manualIntake(m_speed);

    }
    
}
