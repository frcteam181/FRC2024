package frc.robot.commands.Auto;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kDRIVE_TRAIN;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class a_twoPiece extends SequentialCommandGroup {

    private DriveTrain m_driveTrain;
    private Intake m_intake;
    private Arm m_arm;
    private Wrist m_wrist;
    private Flywheel m_Flywheel;

    public a_twoPiece() {

        m_driveTrain = kDRIVE_TRAIN;
        m_intake = kINTAKE;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_Flywheel = kFLYWHEEL;

        addRequirements(m_driveTrain, m_intake, m_Flywheel);

        addCommands(
            
        );

    }
    
}
