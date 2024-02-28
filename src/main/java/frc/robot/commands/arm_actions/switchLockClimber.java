package frc.robot.commands.arm_actions;

import static frc.robot.Constants.kARM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class switchLockClimber extends Command {
    
    private Arm m_arm;

    public switchLockClimber() {

        m_arm = kARM;

    }

    @Override
    public void initialize() {
        m_arm.switchLockClimb();
    }
    
}
