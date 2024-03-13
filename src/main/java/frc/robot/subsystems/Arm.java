package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;



public class Arm extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private SparkAbsoluteEncoder m_encoder;
    private SparkPIDController m_pid;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_armProfile;

    private boolean m_enabled, m_isTuning, m_updateNow, m_isArmSafe, m_isArmSuperSafe;
    private double m_period, m_armFFValue;

    private ArmFeedforward m_armFF;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_posSetpointDeg, e_kP, e_kI, e_kD, e_kKS, e_kKG, e_kKV, e_userGoalDeg;
    private double m_posSetpoint, m_velSetpoint, m_posSetpointDeg, m_kP, m_kI, m_kD, m_kKS, m_kKG, m_kKV, m_userGoal, m_userGoalDeg;
    private double[] m_posResponseDeg, m_velResponseDeg;

    public Arm(boolean isTuning) {

        m_leftMotor = new CANSparkMax(kLEFT_ARM_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(kRIGHT_ARM_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_encoder = m_rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_encoder.setPositionConversionFactor(kARM_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kARM_VEL_FACTOR_RAD); // rad/sec
        m_encoder.setInverted(true);
        m_encoder.setZeroOffset(kARM_ZERO_OFFSET);

        m_pid = m_rightMotor.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);

        m_armFF = new ArmFeedforward(kARM_KS, kARM_KG, kARM_KV); // 0 rad should correspond to parallel to floor
        m_armFFValue = 0.0;

        m_pid.setP(kARM_GAINS.kP, kARM_GAINS.kSlotID);
        m_pid.setI(kARM_GAINS.kI, kARM_GAINS.kSlotID);
        m_pid.setD(kARM_GAINS.kD, kARM_GAINS.kSlotID);
        m_pid.setIZone(kARM_GAINS.kIzone, kARM_GAINS.kSlotID);
        m_pid.setFF(kARM_GAINS.kFF, kARM_GAINS.kSlotID);
        m_pid.setOutputRange(kARM_GAINS.kMinOutput, kARM_GAINS.kMaxOutput, kARM_GAINS.kSlotID);

        m_leftMotor.setSmartCurrentLimit(kLEFT_ARM_CURRENT_LIMIT);
        m_rightMotor.setSmartCurrentLimit(kRIGHT_ARM_CURRENT_LIMIT);

        m_rightMotor.setInverted(true);

        m_leftMotor.follow(m_rightMotor, true);

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        //m_leftMotor.setIdleMode(IdleMode.kCoast);
        //m_rightMotor.setIdleMode(IdleMode.kCoast);

        m_rightMotor.burnFlash();
        m_leftMotor.burnFlash();

        /* Trapezoid Profile */

        m_constraints = new TrapezoidProfile.Constraints(kARM_GAINS.kMaxVel, kARM_GAINS.kMaxAcc); // rad/s & rad/s^2

        m_armProfile = new TrapezoidProfile(m_constraints);

        m_state = new TrapezoidProfile.State(getPos() + kZERO_ARM, 0.0);
        m_goal = new TrapezoidProfile.State(getPos() + kZERO_ARM, 0.0);

        m_enabled  = false;

        m_isArmSafe = false;
        m_isArmSuperSafe = false;

        m_period = 0.02;

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {

        m_state = m_armProfile.calculate(m_period, m_state, m_goal);

        if(getPos() >= kSAFE_ARM_POS_RAD) {m_isArmSafe = true;} else {m_isArmSafe = false;}
        if(getPos() >= kSUPER_SAFE_ARM_POS_RAD) {m_isArmSuperSafe = true;} else {m_isArmSuperSafe = false;}

        if (m_enabled) {useState(m_state);}

        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    
    }

    // Trapezoid Methods

    public void useState(TrapezoidProfile.State state) {
        var err = 2.0;
        m_posSetpoint = (state.position - kZERO_ARM); // Corrected to account for abs encoder not being at exactly zero
        m_velSetpoint = (state.velocity);
        m_armFFValue = m_armFF.calculate(m_posSetpoint, state.velocity);
        //m_pid.setReference(state.position, CANSparkBase.ControlType.kPosition, kARM_GAINS.kSlotID, m_armFFValue);
        if(((getPos() + kZERO_ARM) >= (m_goal.position - Math.toRadians(err))) && ((getPos() + kZERO_ARM) <= (m_goal.position + Math.toRadians(err)))) {m_enabled = false;}
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_state = new TrapezoidProfile.State(getPos() + kZERO_ARM, 0);
        m_enabled = true;
        m_goal = new TrapezoidProfile.State(clamp(goal.position, kMIN_ARM_POS_RAD, kMAX_ARM_POS_RAD) + kZERO_ARM, goal.velocity);
    }

    public void setGoal(double pos) {
        setGoal(new TrapezoidProfile.State(pos, 0.0));
    }

    public Command setGoalCommand(TrapezoidProfile.State goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public Command setGoalCommand(double pos) {
        return Commands.runOnce(() -> setGoal(pos), this);
    }

    public Command setUserGoalCommand() {
        return Commands.runOnce(() -> setGoal(m_userGoal), this);
    }

    public void resetStartPos() {
        m_start = new TrapezoidProfile.State(getPos() + kZERO_ARM, 0);
        m_state = m_start;
    }

    // Class Methods

    public boolean isArmSafe() {
        return m_isArmSafe;
    }

    public boolean isArmSuperSafe() {
        return m_isArmSuperSafe;
    }

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

    public double getPosSetpoint() {
        return m_posSetpoint;
    }

    public double getPosSetpointDeg() {
        return Math.toDegrees(getPosSetpoint());
    }

    public double getPos() {
        return (m_encoder.getPosition() - kZERO_ARM);
    }

    public double getPosDeg() {
        return Math.toDegrees(getPos());
    }

    public double[] getPosResponseDeg() {
        m_posResponseDeg[0] = getPosSetpointDeg();
        m_posResponseDeg[1] = getPosDeg();
        return m_posResponseDeg;
    }

    public double[] getVelResponseDeg() {
        m_velResponseDeg[0] = getVelSetpointDeg();
        m_velResponseDeg[1] = getVel();
        return m_velResponseDeg;
    }

    public double getVelSetpoint() {
        return m_velSetpoint;
    }

    public double getVelSetpointDeg() {
        return Math.toDegrees(getVelSetpoint());
    }

    public double getVel() {
        return m_encoder.getVelocity();
    }

    public double getVelDeg() {
        return Math.toDegrees(getVel());
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

    public void updateGains() {
        m_armFF = new ArmFeedforward(m_kKS, m_kKG, m_kKV);
        m_pid.setP(m_kP, kARM_GAINS.kSlotID);
        m_pid.setI(m_kI, kARM_GAINS.kSlotID);
        m_pid.setD(m_kD, kARM_GAINS.kSlotID);
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

    public double getKKS() {
        return m_kKS;
    }

    public double getKKG() {
        return m_kKG;
    }

    public double getKKV() {
        return m_kKV;
    }

    public double getFFValue() {
        return m_armFFValue;
    }

    public double clamp(double value, double MIN, double MAX) {
        if(value > MAX) {
            return MAX;
        } else if(value < MIN) {
            return MIN;
        }
        return value;
    }

    public boolean isEnabled() {
        return m_enabled;
    }

    public void tune() {

        m_tab = Shuffleboard.getTab("Arm Tuner");

        m_kP = kARM_GAINS.kP;
        m_kI = kARM_GAINS.kI;
        m_kD = kARM_GAINS.kD;

        m_userGoalDeg = 0;

        m_posSetpoint = getPos();
        m_posSetpointDeg = getPosDeg();

        m_velSetpoint = 0;

        m_posResponseDeg = new double[2];
        m_velResponseDeg = new double[2];

        e_kP = m_tab.add("Set kP", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Set kI", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Set kD", m_kD).withPosition(0, 2).getEntry();

        m_tab.addNumber("Kp", this::getKp).withPosition(1, 0);
        m_tab.addNumber("Ki", this::getKi).withPosition(1, 1);
        m_tab.addNumber("Kd", this::getKd).withPosition(1, 2);

        e_userGoalDeg = m_tab.add("User Goal (Deg)", m_userGoalDeg).withPosition(3, 0).getEntry();
        m_tab.addNumber("SetPos (Deg)", this::getPosSetpointDeg).withPosition(4, 0);

        m_tab.addDoubleArray("Response (Deg)", this::getPosResponseDeg).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);
        m_tab.addDoubleArray("Response (Deg:sec)", this::getVelResponseDeg).withPosition(7,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("L.Volts (V)", this::getLeftVoltage).withPosition(2, 1);
        m_tab.addNumber("L.Amps (A)", this::getLeftCurrent).withPosition(2, 2);
        m_tab.addNumber("L.Temp (C)", this::getLeftTemp).withPosition(2, 3);

        // Right Telemetry
        m_tab.addNumber("R.Volts (V)", this::getRightVoltage).withPosition(6, 1);
        m_tab.addNumber("R.Amps (A)", this::getRightCurrent).withPosition(6, 2);
        m_tab.addNumber("R.Temp (C)", this::getRightTemp).withPosition(6, 3);

        // Subsystem Telemetry
        m_tab.addNumber("Pos (deg)", this::getPosDeg).withPosition(5, 0);
        m_tab.addNumber("Vel (deg:sec)", this::getVelDeg).withPosition(8, 0);
        m_tab.addNumber("Set Vel (deg:sec)", this::getVelSetpointDeg).withPosition(7, 0);

        e_kKS = m_tab.add("Set kS", m_kKS).withPosition(10, 0).getEntry();
        e_kKG = m_tab.add("Set kG", m_kKG).withPosition(10, 1).getEntry();
        e_kKV = m_tab.add("Set kV", m_kKV).withPosition(10, 2).getEntry();

        m_tab.addNumber("kS", this::getKKS).withPosition(11, 0);
        m_tab.addNumber("kG", this::getKKG).withPosition(11, 1);
        m_tab.addNumber("kV", this::getKKV).withPosition(11, 2);

        m_tab.addNumber("FF Value", this::getFFValue).withPosition(9, 0);

        m_tab.addBoolean("isEnabled", this::isEnabled).withPosition(6,0).withSize(1, 1);

    }

    public void periodicTuning() {

        var kP = e_kP.getDouble(kARM_GAINS.kP);
        var kI = e_kI.getDouble(kARM_GAINS.kI);
        var kD = e_kD.getDouble(kARM_GAINS.kD);

        var kKS = e_kKS.getDouble(kARM_KS);
        var kKG = e_kKG.getDouble(kARM_KG);
        var kKV = e_kKV.getDouble(kARM_KV);

        if(m_updateNow) {

            if(kP != m_kP) {m_kP = kP;}
            if(kI != m_kI) {m_kI = kI;}
            if(kD != m_kD) {m_kD = kD;}

            if(kKS != m_kKS) {m_kKS = kKS;}
            if(kKG != m_kKG) {m_kKG = kKG;}
            if(kKV != m_kKV) {m_kKV = kKV;}
            
            updateGains();

            m_updateNow = false;
        }

        var userGoalDeg = e_userGoalDeg.getDouble(0);

        if(userGoalDeg != m_userGoalDeg) {m_userGoalDeg = userGoalDeg; m_userGoal = Math.toRadians(userGoalDeg);}

    }
    
}
