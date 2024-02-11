package frc.robot.commands.Arm_C;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class moveArmDown extends Command {

    private Arm m_arm;

    public moveArmDown() {

        m_arm = kARM;

        addRequirements(m_arm);

    }

    @Override
    public void execute() {
        
        m_arm.moveArmDown();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_arm.stopArm();

    }
    
}
