package frc.robot.commands.presets;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm_actions.moveArmTo;
import frc.robot.commands.wrist_actions.moveWristTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class back_high_speaker extends ParallelCommandGroup {

    private Arm m_arm;
    private Wrist m_wrist;

    public back_high_speaker() {

        m_arm = kARM;
        m_wrist = kWRIST;

        addCommands(
            new SequentialCommandGroup(
                new moveArmTo(kBACK_HIGH_SPEAKER_PRESET.kArmPos),
                new moveWristTo(kBACK_HIGH_SPEAKER_PRESET.kWristPos)
            )
        );
    }
    
}
