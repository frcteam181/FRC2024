package frc.robot.commands.Auto;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kBACK_HIGH_SPEAKER_PRESET;
import static frc.robot.Constants.kDRIVE_TRAIN;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kHANDLER;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_PreLoad_n_Taxi extends SequentialCommandGroup {

    private Intake m_intake;
    private Flywheel m_flywheel;
    private Arm m_arm;
    private Wrist m_wrist;
    private Handler m_handler;
    private DriveTrain m_driveTrain;
    
    public a_PreLoad_n_Taxi() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_handler = kHANDLER;
        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_flywheel, m_intake);

        addCommands(
            new ParallelCommandGroup(
                m_handler.setArmCommand(kBACK_HIGH_SPEAKER_PRESET.kArmPos),
                m_handler.setWristCommand(kBACK_HIGH_SPEAKER_PRESET.kWristPos),
                m_handler.setShooterTimerCommand(3)
            ),
            new WaitCommand(3.2),
            m_driveTrain.setSpeedCommand(0.2),
            new WaitCommand(3),
            m_driveTrain.setSpeedCommand(0.0)
        );

    }

}
