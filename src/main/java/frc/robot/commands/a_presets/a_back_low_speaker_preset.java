package frc.robot.commands.a_presets;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kBACK_LOW_SPEAKER_PRESET;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_back_low_speaker_preset extends SequentialCommandGroup {

    private Intake m_intake;
    private Flywheel m_flywheel;
    private Arm m_arm;
    private Wrist m_wrist;
    
    public a_back_low_speaker_preset() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_arm.setGoalCommand(kBACK_LOW_SPEAKER_PRESET.kArmPos),
                m_wrist.setGoalCommand(kBACK_LOW_SPEAKER_PRESET.kWristPos)
            )
        );

    }

}