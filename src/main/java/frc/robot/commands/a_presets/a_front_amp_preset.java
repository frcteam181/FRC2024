package frc.robot.commands.a_presets;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kFORWARD_AMP_PRESET;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_front_amp_preset extends SequentialCommandGroup {

    private Intake m_intake;
    private FlyWheel m_flywheel;
    private Arm m_arm;
    private Wrist m_wrist;
    
    public a_front_amp_preset() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_arm.setGoalCommand(kFORWARD_AMP_PRESET.kArmPos),
                new WaitUntilCommand(m_arm::isArmSafe).andThen(m_wrist.setGoalCommand(kFORWARD_AMP_PRESET.kWristPos))
            )
        );

    }

}