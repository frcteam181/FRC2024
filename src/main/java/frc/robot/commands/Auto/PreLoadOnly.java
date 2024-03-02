package frc.robot.commands.Auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PreLoadOnly extends Command {

    private Arm m_arm;
    private Wrist m_wrist;
    private FlyWheel m_flywheel;
    private Intake m_intake;

    public PreLoadOnly() {

        m_arm = kARM;
        m_wrist = kWRIST;
        m_flywheel = kFLYWHEEL;
        m_intake = kINTAKE;

    }

    @Override
    public void initialize() {
        System.out.println("Pre Load Only Started");
        
        m_arm.resetStartPos();
        m_wrist.resetStartPos();
        m_arm.resetStartPos();
        m_wrist.resetStartPos();

        m_arm.setGoal(kBACK_HIGH_SPEAKER_PRESET.kArmPos);
        m_wrist.setGoal(kBACK_HIGH_SPEAKER_PRESET.kWristPos);
        m_flywheel.setSpeed(50000.0);
    }

    @Override
    public void execute() {

        if(!m_arm.isEnabled() && !m_wrist.isEnabled()) {
            System.out.println("Pre Load Only In Position");
            m_intake.setVel(5000.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_intake.hasNote()) {
            System.out.println("Pre Load Only Shot");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Pre Load Only Ended");
        m_intake.setVel(0.0);
        m_flywheel.setSpeed(0.0);
    }
    
}
