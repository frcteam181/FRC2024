package frc.robot.commands.defaults;

import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class DefaultWrist extends Command {

    private Wrist m_wrist;

    private XboxController m_controller;

    public DefaultWrist(XboxController controller) {

        m_controller = controller;

        m_wrist = kWRIST;

        addRequirements(m_wrist);

    }
    
}
