package frc.robot.commands.intake_actions;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class teleIntake extends Command {

    public enum IntakeState {
        IDLE, INTAKING, ADJUSTING, LOADED, SPEEDING_UP, READY, SHOOTING
    }

    private Intake m_intake;
    private FlyWheel m_flywheel;
    private IntakeState m_intakeState;

    private double m_a, m_b, m_spitDistance;

    public teleIntake() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

        m_a = 0;
        m_b = 0;
        m_spitDistance = 1.5; //in

        addRequirements(m_intake, m_flywheel);

    }

    @Override
    public void initialize() {
        m_intakeState = IntakeState.INTAKING;
    }

    @Override
    public void execute() {

        switch (m_intakeState) {
            case IDLE:
                System.out.println("TeleIntake - IDLE");
                break;

            case INTAKING:
                System.out.println("TeleIntake - INTAKING");
                m_intake.setVel(4000);
                if(m_intake.hasNote()) {
                    //m_intake.setVel(-500);
                    //m_intake.setVelCommand(-500);
                    //m_a = m_intake.getPosIn();
                    m_intakeState = IntakeState.ADJUSTING;
                }
                break;

            case ADJUSTING:
                System.out.println("TeleIntake - ADJUSTING");
                m_b = m_intake.getPosIn();
                if(m_b - m_a >= m_spitDistance) {
                    m_intake.setVel(0);
                    m_intakeState = IntakeState.LOADED;
                }
                break;

            case LOADED: // 1 = Speeding Up
                System.out.println("TeleIntake - LOADED");
                if(m_flywheel.getStatus() == 1) {
                    m_intakeState = IntakeState.SPEEDING_UP;
                }
                break;

            case SPEEDING_UP: // 2 = Ready
                System.out.println("TeleIntake - SPEEDING_UP");
                if(m_flywheel.getStatus() == 2) {
                    m_intakeState = IntakeState.READY;
                } else if (m_flywheel.getStatus() == 0) {
                    m_intakeState = IntakeState.LOADED;
                }
                break;

            case READY:
                System.out.println("TeleIntake - READY");
                if(m_intake.isFeeding()) {
                    m_intakeState = IntakeState.SHOOTING;
                } else if (m_flywheel.getStatus() == 0) {
                    m_intakeState = IntakeState.LOADED;
                } else if (m_flywheel.getStatus() == 1) {
                    m_intakeState = IntakeState.SPEEDING_UP;
                }
                break;

            case SHOOTING:
                System.out.println("TeleIntake - SHOOTING");
                break;
        
        
            default:
                System.out.println("TeleIntake - DEFAULTED");
                break;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.toggleFlywheel();
    }
    
}
