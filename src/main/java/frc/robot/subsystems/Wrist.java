package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase {

    private CANSparkMax m_leftWrist, m_rightWrist;
    private SparkAbsoluteEncoder m_encoder;
    private SparkPIDController m_pid;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_wristProfile;

    private boolean m_enabled, m_isTuning, m_updateNow;

    private double m_period;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_setpointDeg, e_kP, e_kI, e_kD;
    private double m_setpoint, m_setpointDeg, m_kP, m_kI, m_kD;
    private double[] m_responseDeg;

    public Wrist(boolean isTuning) {

        m_leftWrist = new CANSparkMax(kLEFT_WRIST_ID, MotorType.kBrushless);
        m_rightWrist = new CANSparkMax(kRIGHT_WRIST_ID, MotorType.kBrushless);

        m_leftWrist.restoreFactoryDefaults();
        m_rightWrist.restoreFactoryDefaults();

        m_encoder = m_rightWrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_encoder.setPositionConversionFactor(kWRIST_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kWRIST_VEL_FACTOR_RAD); // rad/sec
        m_encoder.setInverted(false);
        m_encoder.setZeroOffset(kWRIST_ZERO_OFFSET);

        m_pid = m_rightWrist.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);

        m_pid.setP(kWRIST_GAINS.kP, kWRIST_PID_SLOT_ID);
        m_pid.setI(kWRIST_GAINS.kI, kWRIST_PID_SLOT_ID);
        m_pid.setD(kWRIST_GAINS.kD, kWRIST_PID_SLOT_ID);
        m_pid.setIZone(kWRIST_GAINS.kIzone, kWRIST_PID_SLOT_ID);
        m_pid.setFF(kWRIST_GAINS.kFF, kWRIST_PID_SLOT_ID);
        m_pid.setOutputRange(kWRIST_GAINS.kMinOutput, kWRIST_GAINS.kMaxOutput, kWRIST_PID_SLOT_ID);

        m_leftWrist.setSmartCurrentLimit(kLEFT_WRIST_CURRENT_LIMIT);
        m_rightWrist.setSmartCurrentLimit(kRIGHT_WRIST_CURRENT_LIMIT);

        m_leftWrist.follow(m_rightWrist, true);

        m_leftWrist.setIdleMode(IdleMode.kBrake);
        m_rightWrist.setIdleMode(IdleMode.kBrake);

        /* Trapezoid Profile */

        m_constraints = new TrapezoidProfile.Constraints(kMAX_WRIST_VEL_RAD, kMAX_WRIST_ACC_RAD); // rad/s & rad/s^2

        m_wristProfile = new TrapezoidProfile(m_constraints);

        m_state = new TrapezoidProfile.State(kZERO_WRIST, 0.0);
        m_goal = new TrapezoidProfile.State(kZERO_WRIST, 0.0);

        m_enabled = true;

        m_period = 0.02;

        /* TUNING */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {

        m_state = m_wristProfile.calculate(m_period, m_state, m_goal);

        if (m_enabled) {useState(m_state);}

        /* TUNING */
        if(m_isTuning) {periodicTuning();}
        
    }

    // Trapezoid Methods

    public void useState(TrapezoidProfile.State state) {
        m_setpoint = (state.position - kZERO_WRIST);
        m_pid.setReference(state.position, CANSparkBase.ControlType.kPosition, kWRIST_PID_SLOT_ID);
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public void setGoal(double pos) {
        m_goal = new TrapezoidProfile.State((pos + kZERO_WRIST), 0.0);
    }


    public Command setGoalCommand(TrapezoidProfile.State goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public Command setGoalCommand(double pos) {
        return Commands.runOnce(() -> setGoal(pos), this);
    }

    // Class Methods

    public void tiltWrist(double speed) {
        m_rightWrist.set(speed);
    }

    public void tiltWristUp() {
        m_rightWrist.set(0.1);
    }

    public void tiltWristDown() {
        m_rightWrist.set(-0.1);
    }

    public void stopWrist() {
        m_rightWrist.set(0.0);
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getSetpointDeg() {
        return Math.toDegrees(getSetpoint());
    }

    public double getPos() {
        return (m_encoder.getPosition() - kZERO_WRIST);
    }

    public double getPosDeg() {
        return Math.toDegrees(getPos());
    }

    public double[] getResponseDeg() {
        m_responseDeg[0] = getSetpointDeg();
        m_responseDeg[1] = getPosDeg();
        return m_responseDeg;
    }

    public double getVel() {
        return m_encoder.getVelocity();
    }

    public double getVelDeg() {
        return Math.toDegrees(getVel());
    }

    public double getLeftVoltage() {
        return (m_leftWrist.getAppliedOutput() * m_leftWrist.getBusVoltage());
    }

    public double getLeftCurrent() {
        return m_leftWrist.getOutputCurrent();
    }

    public double getLeftTemp() {
        return m_leftWrist.getMotorTemperature();
    }

    public double getRightVoltage() {
        return (m_rightWrist.getAppliedOutput() * m_rightWrist.getBusVoltage());
    }

    public double getRightCurrent() {
        return m_rightWrist.getOutputCurrent();
    }

    public double getRightTemp() {
        return m_rightWrist.getMotorTemperature();
    }

    public void updateSetpoint() {
        m_setpoint = Math.toRadians(m_setpointDeg);
    }

    public void updateGains() {
        m_pid.setP(m_kP, kARM_PID_SLOT_ID);
        m_pid.setI(m_kI, kARM_PID_SLOT_ID);
        m_pid.setD(m_kD, kARM_PID_SLOT_ID);
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

    /* TUNING */

    public void tune() {

        m_tab = Shuffleboard.getTab("Wrist Tuner");

        m_kP = kWRIST_GAINS.kP;
        m_kI = kWRIST_GAINS.kI;
        m_kD = kWRIST_GAINS.kD;

        m_setpoint = 0;
        m_setpointDeg = 0;

        m_responseDeg = new double[2];

        e_kP = m_tab.add("Proportional Gain", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Integral Gain", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Derivative Gain", m_kD).withPosition(0, 2).getEntry();

        m_tab.addNumber("Kp", this::getKp).withPosition(1, 0);
        m_tab.addNumber("Ki", this::getKi).withPosition(1, 1);
        m_tab.addNumber("Kd", this::getKd).withPosition(1, 2);

        e_setpointDeg = m_tab.add("Setpoint Deg", m_setpointDeg).withPosition(3, 0).getEntry();
        m_tab.addNumber("Set Setpoint Deg", this::getSetpointDeg).withPosition(4, 0);

        m_tab.addDoubleArray("Response Deg", this::getResponseDeg).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("Left Volts (V)", this::getLeftVoltage).withPosition(2, 1);
        m_tab.addNumber("Left Amps (A)", this::getLeftCurrent).withPosition(2, 2);
        m_tab.addNumber("Left Temp (C)", this::getLeftTemp).withPosition(2, 3);

        // Right Telemetry
        m_tab.addNumber("Right Volts (V)", this::getRightVoltage).withPosition(6, 1);
        m_tab.addNumber("Right Amps (A)", this::getRightCurrent).withPosition(6, 2);
        m_tab.addNumber("Right Temp (C)", this::getRightTemp).withPosition(6, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Position (deg)", this::getPosDeg).withPosition(5, 0);
        m_tab.addNumber("Velocity (deg p sec)", this::getVel).withPosition(6, 0);
        
    }

    public void periodicTuning() {

        var kP = e_kP.getDouble(kWRIST_GAINS.kP);
        var kI = e_kI.getDouble(kWRIST_GAINS.kI);
        var kD = e_kD.getDouble(kWRIST_GAINS.kD);

        if(m_updateNow) {

            if(kP != m_kP) {m_kP = kP;}
            if(kI != m_kI) {m_kI = kI;}
            if(kD != m_kD) {m_kD = kD;}
            
            updateGains();

            m_updateNow = false;
        }

        //var setpointDeg = e_setpointDeg.getDouble(0);

        //if(setpointDeg != m_setpointDeg) {m_setpointDeg = setpointDeg; m_setpoint = Math.toRadians(m_setpointDeg);}

    }
    
}


/* PRESETS!!!


 * Intake 
 *      Arm = 0
 *      Wrist = 0
 * 
 * Forward Amp
 *      Arm = 49
 *      Wrist = -0.7
 * 
 * Backward Amp
 *      Arm = 94.5
 *      Wrist = 5.6
 * 
 * Forward Air Speaker
 *      Arm = 49
 *      Wrist = -110
 * 
 * Backward Air Speaker
 *      Arm = 81
 *      Wrist = -76.9
 * 
 * Backward Ground Speaker
 *      Arm = 16.5
 *      Wrist = -2.4
 */
