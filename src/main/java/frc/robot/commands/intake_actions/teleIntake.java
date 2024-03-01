package frc.robot.commands.intake_actions;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class teleIntake extends Command {

    public enum IntakeState {
        IDLE, INTAKING, LOADED, SPEEDING_UP, READY, SHOOTING
    }

    private Intake m_intake;
    private FlyWheel m_flywheel;
    private IntakeState m_intakeState, m_previousState;
    private Arm m_arm;
    private Wrist m_wrist;

    public teleIntake() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;
        m_arm = kARM;
        m_wrist = kWRIST;

        //addRequirements(m_intake, m_flywheel);

    }

    @Override
    public void initialize() {
        m_intakeState = IntakeState.INTAKING;
        m_previousState = IntakeState.IDLE;
    }

    @Override
    public void execute() {

        switch (m_intakeState) {
            case IDLE:
                System.out.println("TeleIntake - IDLE");
                break;

            case INTAKING:
                System.out.println("TeleIntake - INTAKING");
                m_intake.setVel(kINTAKE_POWER);
                if(m_intake.hasNote()) {
                    m_intake.setVel(0);
                    m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.LOADED;
                }
                break;

            case LOADED: 
                System.out.println("TeleIntake - LOADED");
                if(m_flywheel.getStatus() == 1) {                   // <------ 1 = Speeding Up
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.SPEEDING_UP;
                }
                break;

            case SPEEDING_UP: 
                System.out.println("TeleIntake - SPEEDING_UP");
                if(m_flywheel.getStatus() == 2) {                   // <------ // 2 = Ready
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.READY;
                } else if (m_flywheel.getStatus() == 0) {
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.LOADED;
                }
                break;

            case READY:
                System.out.println("TeleIntake - READY");
                if(m_intake.isFeeding()) {
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.SHOOTING;
                } else if (m_flywheel.getStatus() == 0) {
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.LOADED;
                } else if (m_flywheel.getStatus() == 1) {
                    m_previousState = m_intakeState;
                    m_intakeState = IntakeState.SPEEDING_UP;
                }
                break;

            case SHOOTING:
                System.out.println("TeleIntake - SHOOTING");
                if(!m_intake.hasNote() && m_previousState == IntakeState.READY) {
                    m_flywheel.toggleFlywheel();
                    m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
                    m_intakeState = IntakeState.IDLE;
                }
                break;
        
        
            default:
                System.out.println("TeleIntake - DEFAULTED");
                break;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TeleIntake - ENDED");
        if(m_intakeState == IntakeState.INTAKING) {
            System.out.println("TeleIntake - Intake Stopped");
            m_intake.setVel(0);
        } else if(m_intakeState == IntakeState.SPEEDING_UP || m_intakeState == IntakeState.READY) {
            System.out.println("TeleIntake - Flywheel Stopped");
            m_flywheel.toggleFlywheel();
        }
    }
    
}
