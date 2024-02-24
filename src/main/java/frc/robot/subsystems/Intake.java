package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {

    private CANSparkMax m_motor;

    private RelativeEncoder m_encoder;

    private SparkPIDController m_pid;

    private boolean m_isTuning, m_hasNote, m_updateNow;

    private DigitalInput m_noteBeam;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_setpointDeg, e_kP, e_kI, e_kD, e_kFF;
    private double m_setpoint, m_setpointDeg, m_pidGoal, m_kP, m_kI, m_kD, m_kFF;
    private double[] m_responseDeg;

    public Intake(boolean isTuning) {

        m_motor = new CANSparkMax(kINTAKE_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(kINTAKE_CURRENT_LIMIT);

        m_motor.setInverted(true);

        m_encoder = m_motor.getEncoder();
        //m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kINTAKE_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kINTAKE_VEL_FACTOR_RAD); // rad/sec

        m_pid = m_motor.getPIDController();

        m_pid.setP(kINTAKE_GAINS.kP, kINTAKE_PID_SLOT_ID);
        m_pid.setI(kINTAKE_GAINS.kI, kINTAKE_PID_SLOT_ID);
        m_pid.setD(kINTAKE_GAINS.kD, kINTAKE_PID_SLOT_ID);
        m_pid.setIZone(kINTAKE_GAINS.kIzone, kINTAKE_PID_SLOT_ID);
        m_pid.setFF(kINTAKE_GAINS.kFF, kINTAKE_PID_SLOT_ID);
        m_pid.setOutputRange(kINTAKE_GAINS.kMinOutput, kINTAKE_GAINS.kMaxOutput, kINTAKE_PID_SLOT_ID);

        //m_noteBeam = new DigitalInput(kNOTE_BEAM_CHANNEL);
        //m_hasNote = m_noteBeam.get();

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}
        
    }

    @Override
    public void periodic() {
        m_pid.setReference(m_pidGoal, CANSparkBase.ControlType.kVelocity, kINTAKE_PID_SLOT_ID);
        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    public void manualIntake(double vel) {
        m_motor.set(vel);
    }

    public void setVel(double vel) {
        m_pidGoal = vel;
    }

    public Command setVelCommand(double vel) {
        return Commands.runOnce(() -> setVel(vel), this);
    }

    public Command setTuningVelCommand() {
        return Commands.runOnce(() -> setVel(m_setpoint), this);
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getSetpointDeg() {
        return Math.toDegrees(getSetpoint());
    }

    public double getGoal() {
        return m_pidGoal;
    }

    public double getGoalDeg() {
        return Math.toDegrees(getGoal());
    }

    public double[] getResponseDeg() {
        m_responseDeg[0] = getSetpointDeg();
        m_responseDeg[1] = getVelDeg();
        return m_responseDeg;
    }

    public double getVel() {
        return m_encoder.getVelocity();
    }

    public double getVelDeg() {
        return Math.toDegrees(getVel());
    }

    public double getVoltage() {
        return (m_motor.getAppliedOutput() * m_motor.getBusVoltage());
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public double getTemp() {
        return m_motor.getMotorTemperature();
    }

    public void updateGains() {
        m_pid.setP(m_kP, kINTAKE_PID_SLOT_ID);
        m_pid.setI(m_kI, kINTAKE_PID_SLOT_ID);
        m_pid.setD(m_kD, kINTAKE_PID_SLOT_ID);
        m_pid.setFF(m_kFF, kINTAKE_PID_SLOT_ID);
    }

    public void updateNow() {
        m_updateNow = true;
    }

    public Command updateNowCommand() {
        return Commands.runOnce(() -> updateNow(), this);
    }

    public double getKp() {
        return m_kP;
    }

    public double getKi() {
        return m_kI;
    }

    public double getKd() {
        return m_kD;
    }

    public double getKFF() {
        return m_kFF;
    }

    public void tune() {

        m_tab = Shuffleboard.getTab("Intake Tuner");

        m_kP = kINTAKE_GAINS.kP;
        m_kI = kINTAKE_GAINS.kI;
        m_kD = kINTAKE_GAINS.kD;
        m_kFF = kINTAKE_GAINS.kFF;

        m_setpoint = 0;
        m_setpointDeg = 0;

        m_responseDeg = new double[2];

        e_kP = m_tab.add("Proportional Gain", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Integral Gain", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Derivative Gain", m_kD).withPosition(0, 2).getEntry();
        e_kFF = m_tab.add("Feedforward Gain", m_kFF).withPosition(0, 3).getEntry();

        m_tab.addNumber("kP", this::getKp).withPosition(1, 0);
        m_tab.addNumber("kI", this::getKi).withPosition(1, 1);
        m_tab.addNumber("kD", this::getKd).withPosition(1, 2);
        m_tab.addNumber("kFF", this::getKFF).withPosition(1, 3);

        e_setpointDeg = m_tab.add("Setpoint Deg p sec", m_setpointDeg).withPosition(3, 0).getEntry();
        m_tab.addNumber("Goal Deg p sec", this::getGoalDeg).withPosition(4, 0);

        m_tab.addDoubleArray("Response Deg p sec", this::getResponseDeg).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("Volts (V)", this::getVoltage).withPosition(2, 1);
        m_tab.addNumber("Amps (A)", this::getCurrent).withPosition(2, 2);
        m_tab.addNumber("Temp (C)", this::getTemp).withPosition(2, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Velocity (deg p sec)", this::getVelDeg).withPosition(6, 0);
        
    }

    public void periodicTuning() {

        var kP = e_kP.getDouble(kINTAKE_GAINS.kP);
        var kI = e_kI.getDouble(kINTAKE_GAINS.kI);
        var kD = e_kD.getDouble(kINTAKE_GAINS.kD);
        var kFF = e_kFF.getDouble(kINTAKE_GAINS.kFF);

        if(m_updateNow) {

            if(kP != m_kP) {m_kP = kP;}
            if(kI != m_kI) {m_kI = kI;}
            if(kD != m_kD) {m_kD = kD;}
            if(kFF != m_kFF) {m_kFF = kFF;}
            
            updateGains();

            m_updateNow = false;
        }

        var setpointDeg = e_setpointDeg.getDouble(0);

        if(setpointDeg != m_setpointDeg) {m_setpointDeg = setpointDeg; m_setpoint = Math.toRadians(m_setpointDeg);}

    }
    
}
