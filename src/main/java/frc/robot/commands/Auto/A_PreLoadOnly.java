package frc.robot.commands.Auto;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kBACK_HIGH_SPEAKER_PRESET;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kHANDLER;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_PreLoadOnly extends SequentialCommandGroup {

    private Intake m_intake;
    private Flywheel m_flywheel;
    private Arm m_arm;
    private Wrist m_wrist;
    private Handler m_handler;
    
    public a_PreLoadOnly() {

        m_handler = kHANDLER;

        addRequirements(m_handler);

        addCommands(
            new ParallelCommandGroup(
                m_handler.setArmCommand(kBACK_HIGH_SPEAKER_PRESET.kArmPos),
                m_handler.setWristCommand(kBACK_HIGH_SPEAKER_PRESET.kWristPos),
                m_handler.setShooterTimerCommand(3)
            )//,
            //new WaitCommand(2),
            //m_handler.setIntakeStateCommand(4)
        );

    }

}
