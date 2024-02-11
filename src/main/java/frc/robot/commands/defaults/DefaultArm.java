package frc.robot.commands.defaults;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class DefaultArm extends Command {

    private Arm m_arm;

    private XboxController m_controller;

    private double m_speed;

    public DefaultArm(XboxController controller) {

        m_controller = controller;

        m_arm = kARM;

        addRequirements(m_arm);

    }

    @Override
    public void execute() {
        //m_speed = m_controller.getRightY();
        //m_arm.moveArm(m_speed);
    }


    
}
