package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;



public class Arm extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pid;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_armProfile;

    private boolean m_enabled, m_isTuning;
    private double m_period, m_armFFValue;

    private ArmFeedforward m_armFF;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_setpointDeg, e_kP, e_kI, e_kD;
    private double m_setpoint, m_setpointDeg, m_kP, m_kI, m_kD;
    private double[] m_responseDeg;

    public Arm(boolean isTuning) {

        m_leftMotor = new CANSparkMax(kLEFT_ARM_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(kRIGHT_ARM_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_encoder = m_rightMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        //m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kARM_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kARM_VEL_FACTOR_RAD); // rad/sec

        m_pid = m_rightMotor.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);

        m_armFF = new ArmFeedforward(kARM_KS, kARM_KG, kARM_KV); // 0 rad should correspond to parallel to floor
        m_armFFValue = 0.0;

        m_pid.setP(kARM_GAINS.kP, kARM_PID_SLOT_ID);
        m_pid.setI(kARM_GAINS.kI, kARM_PID_SLOT_ID);
        m_pid.setD(kARM_GAINS.kD, kARM_PID_SLOT_ID);
        m_pid.setIZone(kARM_GAINS.kIzone, kARM_PID_SLOT_ID);
        m_pid.setFF(kARM_GAINS.kFF, kARM_PID_SLOT_ID);
        m_pid.setOutputRange(kARM_GAINS.kMinOutput, kARM_GAINS.kMaxOutput, kARM_PID_SLOT_ID);

        m_leftMotor.setSmartCurrentLimit(kLEFT_ARM_CURRENT_LIMIT);
        m_rightMotor.setSmartCurrentLimit(kRIGHT_ARM_CURRENT_LIMIT);

        m_leftMotor.follow(m_rightMotor, true);

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        /* Trapezoid Profile */

        m_constraints = new TrapezoidProfile.Constraints(kMAX_ARM_VEL_RAD, kMAX_ARM_ACC_RAD); // rad/s & rad/s^2

        m_armProfile = new TrapezoidProfile(m_constraints);

        m_start = new TrapezoidProfile.State(0.0, 0.0);
        m_state = new TrapezoidProfile.State(0.0, 0.0);
        m_goal = new TrapezoidProfile.State(0.0, 0.0);

        m_enabled = false;

        m_period = 0.02;

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {

        m_state = m_armProfile.calculate(m_period, m_state, m_goal);

        if (m_enabled) {
            useState(m_state);
        }

        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    
    }

    // Trapezoid Methods

    public void useState(TrapezoidProfile.State state) {
        m_armFFValue = m_armFF.calculate(state.position, state.velocity);
        m_pid.setReference(state.position, CANSparkBase.ControlType.kPosition, kARM_PID_SLOT_ID, m_armFFValue);
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public void setGoal(double pos) {
        m_goal = new TrapezoidProfile.State(pos, 0.0);
    }

    // Class Methods

    public void moveArm(double speed) {
        m_rightMotor.set(speed);
    }

    public void moveArmUp() {
        m_rightMotor.set(0.1);
    }

    public void moveArmDown() {
        m_rightMotor.set(-0.1);
    }

    public void stopArm() {
        m_rightMotor.set(0.0);
    }

    public double getsetpoint() {
        return m_setpoint;
    }

    public double getsetpointDeg() {
        return Math.toDegrees(getsetpoint());
    }

    public double getPos() {
        return m_encoder.getPosition();
    }

    public double getPosDeg() {
        return Math.toDegrees(getPos());
    }

    public double[] getResponseDeg() {
        m_responseDeg[0] = getsetpointDeg();
        m_responseDeg[1] = getPosDeg();
        return m_responseDeg;
    }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    public double getSpeedDeg() {
        return Math.toDegrees(getSpeed());
    }

    public double getLeftVoltage() {
        return (m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage());
    }

    public double getLeftCurrent() {
        return m_leftMotor.getOutputCurrent();
    }

    public double getLeftTemp() {
        return m_leftMotor.getMotorTemperature();
    }

    public double getRightVoltage() {
        return (m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage());
    }

    public double getRightCurrent() {
        return m_rightMotor.getOutputCurrent();
    }

    public double getRightTemp() {
        return m_rightMotor.getMotorTemperature();
    }

    public void periodicTuning() {

    }

    public void tune() {

        m_tab = Shuffleboard.getTab("Arm Tuner");

        m_kP = kARM_GAINS.kP;
        m_kI = kARM_GAINS.kI;
        m_kD = kARM_GAINS.kD;

        m_setpoint = 0;
        m_setpointDeg = 0;

        m_responseDeg = new double[2];

        e_kP = m_tab.add("Proportional Gain", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Integral Gain", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Derivative Gain", m_kD).withPosition(0, 2).getEntry();

        e_setpointDeg = m_tab.add("Setpoint Deg", m_setpointDeg).withPosition(1, 0).getEntry();

        m_tab.addDoubleArray("Response Deg", this::getResponseDeg).withPosition(2,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("Left Volts (V)", this::getLeftVoltage).withPosition(1, 1);
        m_tab.addNumber("Left Amps (A)", this::getLeftCurrent).withPosition(1, 2);
        m_tab.addNumber("Left Temp (C)", this::getLeftTemp).withPosition(1, 3);

        // Right Telemetry
        m_tab.addNumber("Right Volts (V)", this::getRightVoltage).withPosition(5, 1);
        m_tab.addNumber("Right Amps (A)", this::getRightCurrent).withPosition(5, 2);
        m_tab.addNumber("Right Temp (C)", this::getRightTemp).withPosition(5, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Position (deg)", this::getPosDeg).withPosition(2, 0);
        m_tab.addNumber("Speed (deg p sec)", this::getSpeed).withPosition(3, 0);

    }
    
}
