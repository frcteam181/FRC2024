package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.presets.intake_preset;

import static frc.robot.Constants.*;

public class Handler extends SubsystemBase {

    private enum IntakeState {
        I_IDLE,
        INTAKING,
        ADJUSTING,
        LOADED,
        FEEDING,
        OUTAKING
    }

    private enum FlywheelState {
        F_IDLE,
        SPEEDINGUP,
        READY
    }

    private Arm m_arm;
    private Wrist m_wrist;
    private Intake m_intake;
    private Flywheel m_flywheel;

    private IntakeState m_lastIntakeState, m_currentIntakeState, m_wantedIntakeState, m_lastIntakePrint;
    private FlywheelState m_lastFlywheelState, m_currentFlywheelState, m_wantedFlywheelState, m_lastFlywheelPrint;

    private Timer m_intakeTimer, m_flywheelTimer, m_timeToShoot;

    private double m_timeToShootSec;
    private boolean m_isTimeToShoot;

    public Handler() {

        m_arm = kARM;
        m_wrist = kWRIST;
        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

        // Intake States
        m_lastIntakeState = IntakeState.I_IDLE;
        m_currentIntakeState = IntakeState.I_IDLE;
        m_wantedIntakeState = IntakeState.I_IDLE;

        // Flywheel States
        m_lastFlywheelState = FlywheelState.F_IDLE;
        m_currentFlywheelState = FlywheelState.F_IDLE;
        m_wantedFlywheelState = FlywheelState.F_IDLE;

        m_intakeTimer = new Timer();
        m_flywheelTimer = new Timer();
        m_timeToShoot = new Timer();

        m_timeToShootSec = 0.0;
        m_isTimeToShoot = false;

    }

    @Override
    public void periodic() {

        if(m_timeToShoot.hasElapsed(m_timeToShootSec) && m_isTimeToShoot) {
            setIntakeState(4);
            m_timeToShoot.stop();
            m_isTimeToShoot = false;
        }

        printState();

        switch (m_currentIntakeState) {
            case I_IDLE:
                m_intake.manualIntake(0.0);
                break;

            case INTAKING:
                if(m_lastIntakeState == IntakeState.LOADED && m_intake.hasNote()) {
                    setIntakeState(3);
                } else {
                    if(m_intake.hasNote()) {
                    setIntakeState(2);
                    break;
                    }
                    m_intake.manualIntake(1.0);
                }
                break;

            case ADJUSTING:
                if(m_intakeTimer.hasElapsed(0.1)) {
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
                    setIntakeState(0);
                    setFlywheelState(6);
                } else if(m_intake.hasNote() && m_currentFlywheelState == FlywheelState.READY) {
                    m_intake.manualIntake(0.6);
                } else {
                    setIntakeState(0);
                }
                break;

            case OUTAKING:
                m_intake.manualIntake(-0.5);
                break;
        
            default:
                break;
        }

        switch (m_currentFlywheelState) {
            case F_IDLE:
                m_flywheel.manual(0.0);
                break;

            case SPEEDINGUP:
                if(m_flywheelTimer.hasElapsed(1.5)) {
                    m_flywheelTimer.stop();
                    setFlywheelState(8);
                } else {
                    m_flywheel.manual(1.0);
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
                m_currentIntakeState = IntakeState.I_IDLE;
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
                m_lastIntakeState = m_currentIntakeState;
                m_currentIntakeState = IntakeState.I_IDLE;
                break;
        }
        
    }

    public int getIntakeState() {
        switch (m_currentIntakeState) {
            case I_IDLE:
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

    public void setFlywheelState(int state) {

        switch (state) {
            case 6:
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.F_IDLE;
                break;
                
            case 7:
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.SPEEDINGUP;
                m_flywheelTimer.restart();
                break;
            case 8:
                m_flywheelTimer.stop(); m_flywheelTimer.reset();
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.READY;
                break;
        
            default:
                m_lastFlywheelState = m_currentFlywheelState;
                m_currentFlywheelState = FlywheelState.F_IDLE;
                break;
        }
    }

    public Command setFlywheelStateCommand(int state) {
        return Commands.runOnce(() -> setFlywheelState(state), this);
    }

    public Command setIntakeStateCommand(int state) {
        return Commands.runOnce(() -> setIntakeState(state), this);
    }

    public Command setShooterTimerCommand(double sec) {
        return Commands.runOnce(() -> setShooterTimer(sec), this);
    }

    public void setShooterTimer(double sec) {
        m_timeToShoot.restart();
        setFlywheelState(7);
        m_timeToShootSec = sec;
        m_isTimeToShoot = true;
    }

    public int getFlywheelState() {
        switch (m_currentFlywheelState) {
            case F_IDLE:
                return 6;
            case SPEEDINGUP:
                return 7;
            case READY:
                return 8;
            default:
                return 6;
        }
    }

    public void toggleFlywheel() {
        if(m_currentFlywheelState == FlywheelState.SPEEDINGUP || m_currentFlywheelState == FlywheelState.READY) {
            setFlywheelState(6);
        } else {
            setFlywheelState(7);
        }
    }

    public void toggleIntake() {
        if(m_currentIntakeState == IntakeState.INTAKING) {
            setIntakeState(0);
        } else if(m_currentIntakeState == IntakeState.I_IDLE) {
            setIntakeState(1);
        }
    }

    public void toggleOutake() {
        if(m_currentIntakeState == IntakeState.OUTAKING) {
            setIntakeState(0);
        } else {
            setIntakeState(5);
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

    public Command setArmCommand(double pos) {
        return Commands.runOnce(() -> m_arm.setGoal(pos), m_arm);
    }

    public Command setWristCommand(double pos) {
        return Commands.runOnce(() -> m_wrist.setGoal(pos), m_wrist);
    }
    
    
}
