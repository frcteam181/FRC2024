package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheel;

import static frc.robot.Constants.*;

public class DefaultFlyWheel extends Command {

    private XboxController m_controller;

    private double m_speed;

    private FlyWheel m_flyWheel;

    public DefaultFlyWheel(XboxController controller) {

        m_controller = controller;

        m_flyWheel = kFLYWHEEL;

        addRequirements(m_flyWheel);

    }

    @Override
    public void execute() {
        
        m_speed = m_controller.getRightY();

        m_flyWheel.shoot(m_speed);

    }
    
}
