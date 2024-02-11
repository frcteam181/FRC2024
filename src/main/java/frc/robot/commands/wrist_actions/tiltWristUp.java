package frc.robot.commands.wrist_actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

import static frc.robot.Constants.*;

public class tiltWristUp extends Command {

    private Wrist m_wrist;

    public tiltWristUp() {

        m_wrist = kWRIST;

        addRequirements(m_wrist);

    }

    @Override
    public void execute() {
        m_wrist.tiltWristUp();
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.stopWrist();
    }
    
}
