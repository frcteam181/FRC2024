package frc.robot.commands.Auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Timer;
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
    private Timer m_timer;

    private boolean m_setWrist;

    public PreLoadOnly() {

        m_arm = kARM;
        m_wrist = kWRIST;
        m_flywheel = kFLYWHEEL;
        m_intake = kINTAKE;
        m_timer = new Timer();

        m_setWrist = false;

        addRequirements(m_flywheel, m_intake);

    }

    @Override
    public void initialize() {
        System.out.println("Pre Load Only Started");

        m_arm.setGoal(kBACK_HIGH_SPEAKER_PRESET.kArmPos);

        m_timer.reset();
        m_timer.start();

        m_flywheel.setSpeed(1.0);

    }

    @Override
    public void execute() {

         if(m_arm.isArmSafe() && !m_setWrist) {m_wrist.setGoal(kBACK_HIGH_SPEAKER_PRESET.kWristPos); m_setWrist = true;}

        if(!m_arm.isEnabled() && !m_wrist.isEnabled() && m_timer.hasElapsed(1.8)) {
            System.out.println("Pre Load Only In Position");
            m_intake.setVel(1.0);
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
        m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
        m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
        m_intake.setVel(0.0);
        m_flywheel.setSpeed(0.0);
        m_timer.stop();
    }
    
}
