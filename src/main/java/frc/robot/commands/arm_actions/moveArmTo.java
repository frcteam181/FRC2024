package frc.robot.commands.arm_actions;

import static frc.robot.Constants.kARM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class moveArmTo extends Command {

    private Arm m_arm;
    private double m_goalPos;

    public moveArmTo(double goalPos) {

        m_arm = kARM;
        m_goalPos = goalPos;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        System.out.println("moveArmTo("+ m_goalPos +") initialized");
        m_arm.setGoal(m_goalPos);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("moveArmTo("+ m_goalPos +") ended");
    }

    @Override
    public boolean isFinished() {
        return !m_arm.isEnabled();
    }
    
}
