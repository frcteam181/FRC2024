package frc.robot.commands.wrist_actions;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class moveWristTo extends Command {

    private Wrist m_wrist;
    private double m_goalPos;

    public moveWristTo(double goalPos) {

        m_wrist = kWRIST;
        m_goalPos = goalPos;

        addRequirements(m_wrist);

    }

    @Override
    public void initialize() {
        System.out.println("moveWristTo() initialized");
        m_wrist.setGoal(m_goalPos);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("moveWristTo() ended");
    }

    @Override
    public boolean isFinished() {
        return !m_wrist.isEnabled();
    }
    
}
