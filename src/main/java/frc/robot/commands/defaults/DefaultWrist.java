package frc.robot.commands.defaults;

import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class DefaultWrist extends Command {

    private Wrist m_wrist;

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private boolean m_isPS4;

    public DefaultWrist(XboxController controllerXbox, PS4Controller controllerPS4, boolean isPS4) {

        m_isPS4 = isPS4;

        m_controllerXbox = controllerXbox;
        m_controllerPS4 = controllerPS4;

        m_wrist = kWRIST;

        addRequirements(m_wrist);

    }
    
}
