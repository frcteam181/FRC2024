package frc.robot.commands.arm_actions;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class moveArmUp extends Command {

    private Arm m_arm;

    public moveArmUp() {

        m_arm = kARM;

        addRequirements(m_arm);

    }

    @Override
    public void execute() {
        
        m_arm.moveArmUp();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_arm.stopArm();

    }
    
}
