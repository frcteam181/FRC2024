package frc.robot.commands.Auto;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kDRIVE_TRAIN;
import static frc.robot.Constants.kFLYWHEEL;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kINTAKE_PRESET;
import static frc.robot.Constants.kSTOW_AWAY_PRESET;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PreLoad_n_Taxi extends Command {

    private DriveTrain m_driveTrain;
    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private Flywheel m_flywheel;
    
    private boolean m_hasShot;

    public PreLoad_n_Taxi() {

        m_driveTrain = kDRIVE_TRAIN;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

        m_hasShot = false;

    }

    @Override
    public void execute() {

        if(!m_arm.isEnabled() && !m_wrist.isEnabled() && !m_hasShot) {
            System.out.println("In Position");
            m_intake.setVel(5000.0);
            m_hasShot = true;
        }

        if (!m_intake.hasNote()) {
            System.out.println("Shot now Taxing");

            m_arm.resetStartPos();
            m_wrist.resetStartPos();
            m_arm.resetStartPos();
            m_wrist.resetStartPos();

            m_intake.setVel(0.0);
            m_flywheel.setSpeed(0.0);

            m_arm.setGoal(kINTAKE_PRESET.kArmPos);
            m_wrist.setGoal(kINTAKE_PRESET.kWristPos);

            m_driveTrain.resetEncoders();
            m_driveTrain.setLeftSpeed(0.25);
            m_driveTrain.setRightSpeed(0.25);
        }
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

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
        
    }
    
}
