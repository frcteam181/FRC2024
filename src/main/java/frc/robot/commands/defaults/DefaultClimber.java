package frc.robot.commands.defaults;

import static frc.robot.Constants.kCLIMBER;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class DefaultClimber extends Command {

    private Climber m_climber;

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private double m_speed, m_leftSpeed, m_rightSpeed, m_deadZone;

    private boolean m_isPS4, m_oneStick, m_isLeftStick;

    public DefaultClimber(XboxController controllerXbox, PS4Controller controllerPS4, boolean isPS4, boolean oneStick, boolean isLeftStick) {

        m_isPS4 = isPS4;
        m_oneStick = oneStick;
        m_deadZone = 0.1;
        m_isLeftStick = isLeftStick;
        
        m_controllerXbox = controllerXbox;
        m_controllerPS4 = controllerPS4;

        m_climber = kCLIMBER;

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_climber.setMode(m_oneStick);
    }

    @Override
    public void execute() {
        
        if(m_isPS4) {
            if(m_oneStick) {
                if(m_isLeftStick) {
                    m_speed = m_controllerPS4.getLeftY();
                    m_climber.moveClimber(deadzone(m_speed, m_deadZone));
                } else {
                    m_speed = m_controllerPS4.getRightY();
                    m_climber.moveClimber(deadzone(m_speed, m_deadZone));
                }
            } else {
                m_leftSpeed = m_controllerPS4.getLeftY();
                m_rightSpeed = m_controllerPS4.getRightY();
                m_climber.moveClimber(deadzone(m_leftSpeed, m_deadZone), deadzone(m_rightSpeed, m_deadZone));
            }
        } else {
            if(m_oneStick) {
                if(m_isLeftStick) {
                    m_speed = m_controllerXbox.getLeftY();
                    m_climber.moveClimber(deadzone(m_speed, m_deadZone));
                } else {
                    m_speed = m_controllerXbox.getRightY();
                    m_climber.moveClimber(deadzone(m_speed, m_deadZone));
                }
            } else {
                m_leftSpeed = m_controllerXbox.getLeftY();
                m_rightSpeed = m_controllerXbox.getRightY();
                m_climber.moveClimber(deadzone(m_leftSpeed, m_deadZone), deadzone(m_rightSpeed, m_deadZone));
            }
        }

    }

    public double deadzone(double value, double deadzone) {

        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;

    }
    
}
