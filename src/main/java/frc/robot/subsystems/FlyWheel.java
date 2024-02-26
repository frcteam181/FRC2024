package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class FlyWheel extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;

    private SparkPIDController m_pid;

    private RelativeEncoder m_encoder;

    private boolean m_isTuning, m_updateNow;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_setpoint, e_kP, e_kI, e_kD, e_kFF, e_userSetpoint;
    private double m_setpoint, m_kP, m_kI, m_kD, m_kFF, m_userSetpoint;
    private double[] m_response;

    public FlyWheel(boolean isTuning) {

        m_leftMotor = new CANSparkMax(kLEFT_FLYWHEEL_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(kRIGHT_FLYWHEEL_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.follow(m_rightMotor, true);

        m_encoder = m_rightMotor.getEncoder();
        m_encoder.setPositionConversionFactor(kFLYWHEEL_POS_FACTOR_RAD);//(Units per minute)     // rad
        m_encoder.setVelocityConversionFactor(kFLYWHEEL_VEL_FACTOR_RAD);//(Units per minute)     // rad/sec

        m_pid = m_rightMotor.getPIDController();

        m_pid.setFeedbackDevice(m_encoder);

        m_pid.setP(kFLYWHEEL_GAINS.kP, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setI(kFLYWHEEL_GAINS.kI, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setD(kFLYWHEEL_GAINS.kD, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setIZone(kFLYWHEEL_GAINS.kIzone, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setOutputRange(kFLYWHEEL_GAINS.kMinOutput, kFLYWHEEL_GAINS.kMaxOutput, kFLYWHEEL_GAINS.kSlotID);

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {

        m_pid.setReference(m_setpoint, CANSparkBase.ControlType.kVelocity, kFLYWHEEL_GAINS.kSlotID);

        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    public void shoot(double speed) {
        //m_rightMotor.set(speed);
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

    public double getVel() {
        return m_encoder.getVelocity();
    }

    public double getLeftVoltage() {
        return (m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage());
    }

    public double getRightVoltage() {
        return (m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage());
    }

    public double getLeftCurrent() {
        return m_leftMotor.getOutputCurrent();
    }

    public double getRightCurrent() {
        return m_rightMotor.getOutputCurrent();
    }

    public double getLeftTemp() {
        return m_leftMotor.getMotorTemperature();
    }

    public double getRightTemp() {
        return m_rightMotor.getMotorTemperature();
    }

    public void updateGains() {
        m_pid.setP(m_kP, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setI(m_kI, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setD(m_kD, kFLYWHEEL_GAINS.kSlotID);
        m_pid.setFF(m_kFF, kFLYWHEEL_GAINS.kSlotID);
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

        m_tab = Shuffleboard.getTab("Flywheel Tuner");

        m_kP = kFLYWHEEL_GAINS.kP;
        m_kI = kFLYWHEEL_GAINS.kI;
        m_kD = kFLYWHEEL_GAINS.kD;
        m_kFF = kFLYWHEEL_GAINS.kFF;

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

        e_userSetpoint = m_tab.add("User Setpoint RPM", m_userSetpoint).withPosition(2, 0).withSize(2,1).getEntry();
        m_tab.addNumber("Setpoint RPM", this::getSetpoint).withPosition(4, 0);

        m_tab.addDoubleArray("Response RPM", this::getResponse).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("Left Volts (V)", this::getLeftVoltage).withPosition(2, 1);
        m_tab.addNumber("Left Amps (A)", this::getLeftCurrent).withPosition(2, 2);
        m_tab.addNumber("Left Temp (C)", this::getLeftTemp).withPosition(2, 3);

        // Right Telemetry
        m_tab.addNumber("Right Volts (V)", this::getRightVoltage).withPosition(6, 1);
        m_tab.addNumber("Right Amps (A)", this::getRightCurrent).withPosition(6, 2);
        m_tab.addNumber("Right Temp (C)", this::getRightTemp).withPosition(6, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Velocity RPM", this::getVel).withPosition(5, 0);
        
    }

    public void periodicTuning() {

        var kP = e_kP.getDouble(kFLYWHEEL_GAINS.kP);
        var kI = e_kI.getDouble(kFLYWHEEL_GAINS.kI);
        var kD = e_kD.getDouble(kFLYWHEEL_GAINS.kD);
        var kFF = e_kFF.getDouble(kFLYWHEEL_GAINS.kFF);

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
