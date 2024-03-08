package frc.robot.commands.Auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class TWOPIECE extends Command {

    private Arm m_arm;
    private Wrist m_wrist;
    private Flywheel m_flywheel;
    private Intake m_intake;
    private Timer m_timer;
    private DriveTrain m_driveTrain;

    private boolean m_setWrist, m_noteTwoShot, m_goingToSecond;

    public TWOPIECE() {

        m_driveTrain = kDRIVE_TRAIN;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_flywheel = kFLYWHEEL;
        m_intake = kINTAKE;
        m_timer = new Timer();
        m_noteTwoShot = false;
        m_goingToSecond = false;

        m_setWrist = false;

        addRequirements(m_flywheel, m_intake, m_driveTrain);

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

        if(!m_intake.hasNote() && !m_noteTwoShot && !m_goingToSecond) {

            m_flywheel.setSpeed(0.0);
            m_timer.stop();

            m_wrist.setGoal(kINTAKE_PRESET.kWristPos);
            m_arm.setGoal(kINTAKE_PRESET.kArmPos);

            m_driveTrain.resetEncoders();
            m_driveTrain.setSpeed(0.25);

            m_goingToSecond = true;
            m_timer.reset();
            m_timer.start();
        }

        if((!m_noteTwoShot && m_goingToSecond && m_timer.hasElapsed(1.5)) || (m_intake.hasNote() && m_goingToSecond)) {
            m_timer.stop();
            m_timer.reset();
            m_timer.start();
            m_driveTrain.setSpeed(-0.25);
            m_intake.setVel(0);
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_intake.hasNote() && m_noteTwoShot) {
            System.out.println("Ended Prematurely");
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
        m_timer.stop();
        if(!m_intake.hasNote()) {
        m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
        m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
        }
    }
    
}
