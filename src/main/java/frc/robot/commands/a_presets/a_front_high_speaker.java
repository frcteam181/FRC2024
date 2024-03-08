package frc.robot.commands.a_presets;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm_actions.moveArmTo;
import frc.robot.commands.wrist_actions.moveWristTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_front_high_speaker extends ParallelCommandGroup {

    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private Flywheel m_flywheel;

    public a_front_high_speaker() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_arm.setGoalCommand(kFORWARD_HIGH_SPEAKER_PRESET.kArmPos),
                new WaitUntilCommand(m_arm::isArmSafe).andThen(m_wrist.setGoalCommand(kFORWARD_HIGH_SPEAKER_PRESET.kWristPos))
            )
        );
    }
    
}
