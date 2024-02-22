package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;

    private RelativeEncoder m_leftEncoder, m_rightEncoder;

    private SparkPIDController m_leftPID, m_rightPID;

    private DifferentialDrive m_diffDrive;

    private boolean m_isTuning;

    // Tuning Param
    private ShuffleboardTab m_tab;
    private GenericEntry e_leftSetpointIn, e_rightSetpointIn, e_kP, e_kI, e_kD;
    private double m_leftSetpoint, m_rightSetpoint, m_leftSetpointIn, m_rightSetpointIn, m_kP, m_kI, m_kD;
    private double[] m_leftResponseIn, m_rightResponseIn;

    

    public DriveTrain(boolean isTuning) {
    
        m_leftLeader = new CANSparkMax(kLEFT_LEADER_ID, MotorType.kBrushless);
        m_leftFollower = new CANSparkMax(kLEFT_FOLLOWER_ID, MotorType.kBrushless);
        m_rightLeader = new CANSparkMax(kRIGHT_LEADER_ID, MotorType.kBrushless);
        m_rightFollower = new CANSparkMax(kRIGHT_FOLLOWER_ID, MotorType.kBrushless);

        m_leftLeader.restoreFactoryDefaults();
        m_leftFollower.restoreFactoryDefaults();
        m_rightLeader.restoreFactoryDefaults();
        m_rightFollower.restoreFactoryDefaults();

        m_leftLeader.setSmartCurrentLimit(kLEFT_DRIVETRAIN_CURRENT_LIMIT);
        m_rightLeader.setSmartCurrentLimit(kLEFT_DRIVETRAIN_CURRENT_LIMIT);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        m_leftLeader.setInverted(true);
        m_rightLeader.setInverted(false);

        m_leftLeader.setIdleMode(IdleMode.kBrake);
        m_leftFollower.setIdleMode(IdleMode.kBrake);
        m_rightLeader.setIdleMode(IdleMode.kBrake);
        m_rightFollower.setIdleMode(IdleMode.kBrake);

        m_leftEncoder = m_leftLeader.getEncoder();
        //m_leftEncoder.setInverted(false);
        m_leftEncoder.setPositionConversionFactor(kDRIVETRAIN_POS_FACTOR_METER); // m
        m_leftEncoder.setVelocityConversionFactor(kDRIVETRAIN_VEL_FACTOR_METER); // m/sec

        m_rightEncoder = m_rightLeader.getEncoder();
        //m_rightEncoder.setInverted(false);
        m_rightEncoder.setPositionConversionFactor(kDRIVETRAIN_POS_FACTOR_METER); // m
        m_rightEncoder.setVelocityConversionFactor(kDRIVETRAIN_VEL_FACTOR_METER); // m/sec

        m_leftPID = m_leftLeader.getPIDController();
        m_rightPID = m_rightLeader.getPIDController();

        m_leftPID.setP(kDRIVE_GAINS.kP, kDRIVE_PID_SLOT_ID);
        m_leftPID.setI(kDRIVE_GAINS.kI, kDRIVE_PID_SLOT_ID);
        m_leftPID.setD(kDRIVE_GAINS.kD, kDRIVE_PID_SLOT_ID);
        m_leftPID.setIZone(kDRIVE_GAINS.kIzone, kDRIVE_PID_SLOT_ID);
        m_leftPID.setFF(kDRIVE_GAINS.kFF, kDRIVE_PID_SLOT_ID);
        m_leftPID.setOutputRange(kDRIVE_GAINS.kMinOutput, kDRIVE_GAINS.kMaxOutput, kDRIVE_PID_SLOT_ID);

        m_rightPID.setP(kDRIVE_GAINS.kP, kDRIVE_PID_SLOT_ID);
        m_rightPID.setI(kDRIVE_GAINS.kI, kDRIVE_PID_SLOT_ID);
        m_rightPID.setD(kDRIVE_GAINS.kD, kDRIVE_PID_SLOT_ID);
        m_rightPID.setIZone(kDRIVE_GAINS.kIzone, kDRIVE_PID_SLOT_ID);
        m_rightPID.setFF(kDRIVE_GAINS.kFF, kDRIVE_PID_SLOT_ID);
        m_rightPID.setOutputRange(kDRIVE_GAINS.kMinOutput, kDRIVE_GAINS.kMaxOutput, kDRIVE_PID_SLOT_ID);
        m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {
        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    // To default the values to use direct joystick values, change n = 1; and kDRIVE_DEADZONE to 0;
    public void useTankDrive(double leftValue, double rightValue, boolean squareInput) {
        m_diffDrive.tankDrive(deadzone(applyCurve(-leftValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(-rightValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), squareInput);
    }

    public void useArcadeDrive(double speedValue, double turnValue, boolean squareInput) {
        m_diffDrive.arcadeDrive(deadzone(applyCurve(-speedValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(-turnValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), squareInput);
    }

    /*
     *  O is the output to the motors
     *  I is the input for the joystick
     *  n is the odd number that determines how much curve is stretched toward the lower values
     *  T is the threshold after which the robot will start to move
     */
    public double applyCurve(double joystickInput, int n) {

        if (joystickInput > 0) {

            return (((1 - kDRIVE_THRESHOLD) * Math.pow(joystickInput, n)) + kDRIVE_THRESHOLD);

        } else if (joystickInput < 0) {

            return (((1 - kDRIVE_THRESHOLD) * Math.pow(joystickInput, n)) - kDRIVE_THRESHOLD);

        }

        return 0;
    }

    public double deadzone(double value, double deadzone) {

        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;

    }

    public double getLeftSetpoint() {
        return m_leftSetpoint;
    }

    public double getRightSetpoint() {
        return m_rightSetpoint;
    }

    public double getLeftSetpointIn() {
        return Units.metersToInches(getLeftSetpoint());
    }

    public double getRightSetpointIn() {
        return Units.metersToInches(getRightSetpoint());
    }

    public double getLeftPos() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPos() {
        return m_rightEncoder.getPosition();
    }

    public double getLeftPosIn() {
        return Units.metersToInches(getLeftPos());
    }

    public double getRightPosIn() {
        return Units.metersToInches(getRightPos());
    }

    public double[] getLeftResponseIn() {
        m_leftResponseIn[0] = getLeftSetpointIn();
        m_leftResponseIn[1] = getLeftPosIn();
        return m_leftResponseIn;
    }

    public double[] getRightResponseIn() {
        m_rightResponseIn[0] = getRightSetpointIn();
        m_rightResponseIn[1] = getRightPosIn();
        return m_rightResponseIn;
    }

    public double getLeftSpeed() {
        return m_leftEncoder.getVelocity();
    }

    public double getLeftSpeedIn() {
        return Units.metersToInches(getLeftSpeed());
    }

    public double getRightSpeed() {
        return m_rightEncoder.getVelocity();
    }

    public double getRightSpeedIn() {
        return Units.metersToInches(getRightSpeed());
    }

    public double getLeftVoltage() {
        return (m_leftLeader.getAppliedOutput() * m_leftLeader.getBusVoltage());
    }

    public double getRightVoltage() {
        return (m_rightLeader.getAppliedOutput() * m_rightLeader.getBusVoltage());
    }

    public double getLeftCurrent() {
        return m_leftLeader.getOutputCurrent();
    }

    public double getLeftTemp() {
        return m_leftLeader.getMotorTemperature();
    }

    public double getRightCurrent() {
        return m_rightLeader.getOutputCurrent();
    }

    public double getRightTemp() {
        return m_rightLeader.getMotorTemperature();
    }

    public void periodicTuning() {

    }

    public void tune() {

        m_tab = Shuffleboard.getTab("DriveTrain Tuner");

        m_kP = kDRIVE_GAINS.kP;
        m_kI = kDRIVE_GAINS.kI;
        m_kD = kDRIVE_GAINS.kD;

        m_leftSetpoint = 0;
        m_leftSetpointIn = 0;

        m_rightSetpoint = 0;
        m_rightSetpointIn = 0;

        m_leftResponseIn = new double[2];
        m_rightResponseIn = new double[2];

        e_kP = m_tab.add("Proportional Gain", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Integral Gain", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Derivative Gain", m_kD).withPosition(0, 2).getEntry();

        e_leftSetpointIn = m_tab.add("Left Setpoint In", m_leftSetpointIn).withPosition(1, 0).getEntry();
        e_rightSetpointIn = m_tab.add("Right Setpoint In", m_rightSetpointIn).withPosition(1, 0).getEntry();

        m_tab.addDoubleArray("Left Response In", this::getLeftResponseIn).withPosition(2,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);
        m_tab.addDoubleArray("Right Response In", this::getRightResponseIn).withPosition(2,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("Left Volts (V)", this::getLeftVoltage).withPosition(1, 1);
        m_tab.addNumber("Left Amps (A)", this::getLeftCurrent).withPosition(1, 2);
        m_tab.addNumber("Left Temp (C)", this::getLeftTemp).withPosition(1, 3);
        m_tab.addNumber("Left Position (in)", this::getLeftPosIn).withPosition(2, 0);
        m_tab.addNumber("Left Speed (in p sec)", this::getLeftSpeedIn).withPosition(3, 0);

        // Right Telemetry
        m_tab.addNumber("Right Volts (V)", this::getRightVoltage).withPosition(5, 1);
        m_tab.addNumber("Right Amps (A)", this::getRightCurrent).withPosition(5, 2);
        m_tab.addNumber("Right Temp (C)", this::getRightTemp).withPosition(5, 3);
        m_tab.addNumber("Right Position (in)", this::getRightPosIn).withPosition(2, 0);
        m_tab.addNumber("Right Speed (in p sec)", this::getRightSpeedIn).withPosition(3, 0);
        
    }
    
}