package frc.robot.commands.a_presets;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_intake_preset extends ParallelCommandGroup {

    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private Flywheel m_flywheel;

    public a_intake_preset() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_arm.setGoalCommand(kINTAKE_PRESET.kArmPos),
                m_wrist.setGoalCommand(kINTAKE_PRESET.kWristPos)
                /*m_wrist.setGoalCommand(kINTAKE_PRESET.kWristPos),
                new WaitUntilCommand(m_arm::isArmSuperSafe).andThen(m_arm.setGoalCommand(kINTAKE_PRESET.kArmPos))*/
            )
        );
    }
    
}
