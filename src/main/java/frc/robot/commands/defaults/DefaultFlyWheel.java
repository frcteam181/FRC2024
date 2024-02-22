package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheel;

import static frc.robot.Constants.*;

public class DefaultFlyWheel extends Command {

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private double m_speed;

    private boolean m_isPS4;

    private FlyWheel m_flyWheel;

    public DefaultFlyWheel(XboxController controllerXbox, PS4Controller controllerPS4, boolean isPS4) {

        m_isPS4 = isPS4;

        m_controllerXbox = controllerXbox;
        m_controllerPS4 = controllerPS4;

        m_flyWheel = kFLYWHEEL;

        addRequirements(m_flyWheel);

    }

    @Override
    public void execute() {
        
        if(m_isPS4) {
            m_speed = m_controllerPS4.getRightY();
        } else {
            m_speed = m_controllerXbox.getRightY();
        }
        m_flyWheel.shoot(m_speed);

    }
    
}
