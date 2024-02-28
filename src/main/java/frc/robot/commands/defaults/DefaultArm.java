package frc.robot.commands.defaults;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class DefaultArm extends Command {

    private Arm m_arm;

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private double m_speed;

    private boolean m_isPS4;

    public DefaultArm(XboxController controllerXbox, PS4Controller controllerPS4, boolean isPS4) {

        m_controllerXbox = controllerXbox;
        m_controllerPS4 = controllerPS4;

        m_isPS4 = isPS4;

        m_arm = kARM;

        addRequirements(m_arm);

    }

    @Override
    public void execute() {
        //m_speed = m_controllerPS4.getRightY();
        //m_arm.moveArm(m_speed);
    }
    
}
