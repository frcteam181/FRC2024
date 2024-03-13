package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.presets.intake_preset;

import static frc.robot.Constants.*;

public class Handler extends SubsystemBase {

    private enum IntakeState {
        IDLE,
        INTAKING,
        ADJUSTING,
        LOADED,
        FEEDING,
        OUTAKING
    }

    private enum FlywheelState {
        IDLE,
        SPEEDINGUP,
        READY
    }

    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private Flywheel m_flywheel;

    private IntakeState m_lastIntakeState, m_currentIntakeState, m_wantedIntakeState, m_lastIntakePrint;
    private FlywheelState m_lastFlywheelState, m_currentFlywheelState, m_wantedFlywheelState, m_lastFlywheelPrint;

    private Timer m_intakeTimer, m_flywheelTimer;

    public Handler() {

        m_arm = kARM;
        m_wrist = kWRIST;
        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

        // Intake States
        m_lastIntakeState = IntakeState.IDLE;
        m_currentIntakeState = IntakeState.IDLE;
        m_wantedIntakeState = IntakeState.IDLE;

        // Flywheel States
        m_lastFlywheelState = FlywheelState.IDLE;
        m_currentFlywheelState = FlywheelState.IDLE;
        m_wantedFlywheelState = FlywheelState.IDLE;

        m_intakeTimer = new Timer();
        m_flywheelTimer = new Timer();

    }

    @Override
    public void periodic() {

        printState();

        switch (m_currentIntakeState) {
            case IDLE:
                m_intake.manualIntake(0.0);
                break;

            case INTAKING:
                if(m_lastIntakeState == IntakeState.LOADED && m_intake.hasNote()) {
                    setIntakeState(3);
                } else {
                    m_intake.manualIntake(1.0);
                    if(m_intake.hasNote()) {
                    setIntakeState(2);
                }
                }
                break;

            case ADJUSTING:
                if(m_intakeTimer.hasElapsed(1.0)) {
                    m_intakeTimer.stop(); m_intakeTimer.reset();
                    m_lastIntakeState = m_currentIntakeState;
                    m_currentIntakeState = IntakeState.LOADED;
                } else {
                    m_intake.manualIntake(-0.2);
                }
                
                break;

            case LOADED:
                m_intake.manualIntake(0.0);
                break;

            case FEEDING:
                if(!m_intake.hasNote()) {
                    m_intake.manualIntake(0.0);
                } else {
                    m_intake.manualIntake(kINTAKE_FEED_POWER);
                }
                break;

            case OUTAKING:
                m_intake.manualIntake(-1.0);
                break;
        
            default:
                break;
        }

        switch (m_currentFlywheelState) {
            case IDLE:
                m_flywheel.setVel(0.0);
                break;

            case SPEEDINGUP:
                if(m_flywheelTimer.hasElapsed(1.5)) {
                    m_flywheelTimer.stop();
                    setFlywheelState(2);
                } else {
                    m_flywheel.setVel(1.0);
                }
                break;

            case READY:

                break;
        
            default:
                break;
        }

    }

    public void setIntakeState(int state) {

        switch (state) {
            case 0:
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.IDLE;
                break;
            case 1:
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.INTAKING;
                break;
            case 2:
                m_lastIntakeState = m_currentIntakeState; 
                m_currentIntakeState = IntakeState.ADJUSTING; 
                m_intakeTimer.restart();
                m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
                m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
                break;
            case 3:
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.LOADED;
                break;
            case 4:
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.FEEDING;
                break;
            case 5:
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.OUTAKING;
                break;
        
            default:
                break;
        }
        
    }

    public int getIntakeState() {
        switch (m_currentIntakeState) {
            case IDLE:
                return 0;
            case INTAKING:
                return 1;
            case ADJUSTING:
                return 2;
            case LOADED:
                return 3;
            case FEEDING:
                return 4;
            case OUTAKING:
                return 5;
        
            default:
                return 0;
        }
    }

    public int getFlywheelState() {
        switch (m_currentFlywheelState) {
            case IDLE:
                return 6;
            case SPEEDINGUP:
                return 7;
            case READY:
                return 8;
            default:
                return 6;
        }
    }

    public void setFlywheelState(int state) {

        switch (state) {
            case 0:
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.IDLE;
                break;
            case 1:
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.SPEEDINGUP;
                m_flywheelTimer.restart();
                break;
            case 2:
                m_flywheelTimer.stop(); m_flywheelTimer.reset();
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.READY;
                break;
        
            default:
                break;
        }
    }

    public void toggleFlywheel() {
        if(m_currentFlywheelState == FlywheelState.SPEEDINGUP || m_currentFlywheelState == FlywheelState.READY) {
            setFlywheelState(0);
        } else {
            setFlywheelState(1);
        }
    }

    public void printState() {
        if(m_currentIntakeState != m_lastIntakePrint) {
            System.out.println(m_currentIntakeState.toString());
            m_lastIntakePrint = m_currentIntakeState;
        }

        if(m_currentFlywheelState != m_lastFlywheelPrint) {
            System.out.println(m_currentFlywheelState.toString());
            m_lastFlywheelPrint = m_currentFlywheelState;
        }
    }
    
    
}
