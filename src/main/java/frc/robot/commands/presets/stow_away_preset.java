package frc.robot.commands.presets;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm_actions.moveArmTo;
import frc.robot.commands.wrist_actions.moveWristTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class stow_away_preset extends ParallelCommandGroup {

    private Arm m_arm;
    private Wrist m_wrist;

    public stow_away_preset() {

        m_arm = kARM;
        m_wrist = kWRIST;

        addCommands(
            new SequentialCommandGroup(
                new moveArmTo(kSTOW_AWAY_PRESET.kArmPos),
                new moveWristTo(kSTOW_AWAY_PRESET.kWristPos)
            )
        );
    }
    
}
