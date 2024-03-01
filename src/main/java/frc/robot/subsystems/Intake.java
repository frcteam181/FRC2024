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

    private boolean m_isTuning, m_hasNote, m_updateNow, m_isFeeding;

    private DigitalInput m_noteBeam;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_kP, e_kI, e_kD, e_kFF, e_userSetpoint;
    private double m_setpoint, m_kP, m_kI, m_kD, m_kFF, m_userSetpoint;
    private double[] m_response;

    public Intake(boolean isTuning) {

        m_motor = new CANSparkMax(kINTAKE_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(kINTAKE_CURRENT_LIMIT);

        m_motor.setInverted(true);

        m_encoder = m_motor.getEncoder();
        //m_encoder.setPositionConversionFactor(kINTAKE_POS_FACTOR_RAD); // rad
        //m_encoder.setVelocityConversionFactor(kINTAKE_VEL_FACTOR_RAD); // rad/sec

        m_pid = m_motor.getPIDController();

        m_pid.setP(kINTAKE_GAINS.kP, kINTAKE_GAINS.kSlotID);
        m_pid.setI(kINTAKE_GAINS.kI, kINTAKE_GAINS.kSlotID);
        m_pid.setD(kINTAKE_GAINS.kD, kINTAKE_GAINS.kSlotID);
        m_pid.setIZone(kINTAKE_GAINS.kIzone, kINTAKE_GAINS.kSlotID);
        m_pid.setFF(kINTAKE_GAINS.kFF, kINTAKE_GAINS.kSlotID);
        m_pid.setOutputRange(kINTAKE_GAINS.kMinOutput, kINTAKE_GAINS.kMaxOutput, kINTAKE_GAINS.kSlotID);

        m_noteBeam = new DigitalInput(1);
        m_hasNote = m_noteBeam.get();
        m_isFeeding = false;

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}
        
    }

    @Override
    public void periodic() {
        if(!m_isFeeding) {
            //m_pid.setReference(m_setpoint, CANSparkBase.ControlType.kVelocity, kINTAKE_GAINS.kSlotID);
            m_motor.set(m_setpoint);
        } else {
            m_motor.set(1);
            //m_pid.setReference(kINTAKE_FEED_POWER, CANSparkBase.ControlType.kVelocity, kINTAKE_GAINS.kSlotID);
        }
        m_hasNote = m_noteBeam.get();
        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    public boolean hasNote() {
        return !m_hasNote;
    }

    public void manualIntake(double vel) {
        m_motor.set(vel);
    }

    public void setVel(double RPM) {
        m_setpoint = RPM;
    }

    public Command setVelCommand(double RPM) {
        return Commands.runOnce(() -> setVel(RPM), this);
    }

    public Command setUserSetpointCommand() {
        return Commands.runOnce(() -> setVel(m_userSetpoint), this);
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double[] getResponse() {
        m_response[0] = getSetpoint();
        m_response[1] = getVel();
        return m_response;
    }

    public double getPos() {
        return m_encoder.getPosition();
    }

    public double getPosIn() {
        return (getPos() * (1.0/25.0) * (2 * Math.PI));
    }

    public double getVel() {
        return m_encoder.getVelocity();
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

    public boolean isFeeding() {
        return m_isFeeding;
    }

    public void toggleFeed() {
        m_isFeeding = !m_isFeeding;
    }

    public void updateGains() {
        m_pid.setP(m_kP, kINTAKE_GAINS.kSlotID);
        m_pid.setI(m_kI, kINTAKE_GAINS.kSlotID);
        m_pid.setD(m_kD, kINTAKE_GAINS.kSlotID);
        m_pid.setFF(m_kFF, kINTAKE_GAINS.kSlotID);
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
        m_userSetpoint = 0;

        m_response = new double[2];

        e_kP = m_tab.add("Proportional Gain", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Integral Gain", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Derivative Gain", m_kD).withPosition(0, 2).getEntry();
        e_kFF = m_tab.add("Feedforward Gain", m_kFF).withPosition(0, 3).getEntry();

        m_tab.addNumber("kP", this::getKp).withPosition(1, 0);
        m_tab.addNumber("kI", this::getKi).withPosition(1, 1);
        m_tab.addNumber("kD", this::getKd).withPosition(1, 2);
        m_tab.addNumber("kFF", this::getKFF).withPosition(1, 3);

        m_tab.addBoolean("Has Note", this::hasNote).withPosition(6, 0).withSize(1, 1);

        e_userSetpoint = m_tab.add("User Setpoint RPM", m_userSetpoint).withPosition(2, 0).withSize(2, 1).getEntry();
        m_tab.addNumber("Setpoint RPM", this::getSetpoint).withPosition(4, 0);

        m_tab.addDoubleArray("Response RPM", this::getResponse).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        //  Telemetry
        m_tab.addNumber(" Volts (V)", this::getVoltage).withPosition(2, 1);
        m_tab.addNumber(" Amps (A)", this::getCurrent).withPosition(2, 2);
        m_tab.addNumber(" Temp (C)", this::getTemp).withPosition(2, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Velocity RPM", this::getVel).withPosition(5, 0);
        
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

        var userSetpoint = e_userSetpoint.getDouble(0);

        if(userSetpoint != m_userSetpoint) {m_userSetpoint = userSetpoint;}

    }
    
}
