package frc.robot.commands.a_presets;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_stow_away_preset extends ParallelCommandGroup {

    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private FlyWheel m_flywheel;

    public a_stow_away_preset() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_arm.setGoalCommand(kSTOW_AWAY_PRESET.kArmPos),
                new WaitUntilCommand(m_arm::isArmSafe).andThen(m_wrist.setGoalCommand(kSTOW_AWAY_PRESET.kWristPos))
            )
        );
    }
    
}
