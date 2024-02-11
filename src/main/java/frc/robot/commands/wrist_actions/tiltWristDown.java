package frc.robot.commands.wrist_actions;

import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class tiltWristDown extends Command {

    private Wrist m_wrist;

    public tiltWristDown() {

        m_wrist = kWRIST;

        addRequirements(m_wrist);

    }

    @Override
    public void execute() {
        m_wrist.tiltWristDown();
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.stopWrist();
    }
    
}
