package frc.robot.commands.Auto;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kDRIVE_TRAIN;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kSTOW_AWAY_PRESET;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class TaxiOnly extends Command {

    private DriveTrain m_driveTrain;
    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private FlyWheel m_flywheel;

    public TaxiOnly() {

        m_driveTrain = kDRIVE_TRAIN;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

    }

    @Override
    public void initialize() {

        m_arm.resetStartPos();
        m_wrist.resetStartPos();
        m_arm.resetStartPos();
        m_wrist.resetStartPos();

        m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
        m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
        m_driveTrain.setLeftSpeed(0.25);
        m_driveTrain.setRightSpeed(0.25);
    }

    @Override
    public boolean isFinished() {
        if (m_driveTrain.getLeftPos() >= Units.inchesToMeters(48.0) && m_driveTrain.getRightPos() >= Units.inchesToMeters(48.0)) {
            System.out.println("Taxied");
            m_driveTrain.stop();
            return true;
        } else {
            return false;
        }
    }
    
}
