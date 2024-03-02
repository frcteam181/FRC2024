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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;

    private RelativeEncoder m_leftEncoder, m_rightEncoder;

    private SparkPIDController m_leftPID, m_rightPID;

    private DifferentialDrive m_diffDrive;

    private boolean m_isTuning, m_DRIVE_POS, m_DRIVE_VEL, m_TURN_POS, m_TURN_VEL, m_updateNow;

    //private PigeonIMU m_gyro;

    // Tuning Parameters
    private ShuffleboardTab m_tab;
    private GenericEntry e_leftSetpointPosIn, e_rightSetpointPosIn, e_leftSetpointVelIn, e_rightSetpointVelIn, e_kP, e_kI, e_kD, e_kFF;
    private double m_leftPosSetpoint, m_rightPosSetpoint, m_leftPosSetpointIn, m_rightPosSetpointIn, m_leftVelSetpoint, m_rightVelSetpoint, m_leftVelSetpointIn, m_rightVelSetpointIn, m_leftVel, m_rightVel, m_leftVelIn, m_rightVelIn, m_kP, m_kI, m_kD, m_kFF;
    private double[] m_leftPosResponse, m_rightPosResponse, m_leftPosResponseIn, m_rightPosResponseIn, m_leftVelResponse, m_rightVelResponse, m_leftVelResponseIn, m_rightVelResponseIn;

    

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
        m_leftFollower.setSmartCurrentLimit(kLEFT_DRIVETRAIN_CURRENT_LIMIT);
        m_rightLeader.setSmartCurrentLimit(kRIGHT_DRIVETRAIN_CURRENT_LIMIT);
        m_rightFollower.setSmartCurrentLimit(kRIGHT_DRIVETRAIN_CURRENT_LIMIT);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        m_leftLeader.setInverted(false);
        m_rightLeader.setInverted(true);

        m_leftLeader.setIdleMode(IdleMode.kBrake);
        m_leftFollower.setIdleMode(IdleMode.kBrake);
        m_rightLeader.setIdleMode(IdleMode.kBrake);
        m_rightFollower.setIdleMode(IdleMode.kBrake);

        m_leftEncoder = m_leftLeader.getEncoder();
        m_leftEncoder.setPositionConversionFactor(kDRIVETRAIN_POS_FACTOR_METER); // m
        m_leftEncoder.setVelocityConversionFactor(kDRIVETRAIN_VEL_FACTOR_METER); // m/sec

        m_rightEncoder = m_rightLeader.getEncoder();
        m_rightEncoder.setPositionConversionFactor(kDRIVETRAIN_POS_FACTOR_METER); // m
        m_rightEncoder.setVelocityConversionFactor(kDRIVETRAIN_VEL_FACTOR_METER); // m/sec

        m_leftLeader.setOpenLoopRampRate(0.35);
        m_rightLeader.setOpenLoopRampRate(0.35);

        m_leftPID = m_leftLeader.getPIDController();
        m_rightPID = m_rightLeader.getPIDController();

        // Position PID

        // DRIVE
        m_leftPID.setP(kDT_DRIVE_POS_GAINS.kP, kDT_DRIVE_POS_GAINS.kSlotID);
        m_leftPID.setI(kDT_DRIVE_POS_GAINS.kI, kDT_DRIVE_POS_GAINS.kSlotID);
        m_leftPID.setD(kDT_DRIVE_POS_GAINS.kD, kDT_DRIVE_POS_GAINS.kSlotID);
        m_leftPID.setIZone(kDT_DRIVE_POS_GAINS.kIzone, kDT_DRIVE_POS_GAINS.kSlotID);
        m_leftPID.setFF(kDT_DRIVE_POS_GAINS.kFF, kDT_DRIVE_POS_GAINS.kSlotID);
        m_leftPID.setOutputRange(kDT_DRIVE_POS_GAINS.kMinOutput, kDT_DRIVE_POS_GAINS.kMaxOutput, kDT_DRIVE_POS_GAINS.kSlotID);

        m_rightPID.setP(kDT_DRIVE_POS_GAINS.kP, kDT_DRIVE_POS_GAINS.kSlotID);
        m_rightPID.setI(kDT_DRIVE_POS_GAINS.kI, kDT_DRIVE_POS_GAINS.kSlotID);
        m_rightPID.setD(kDT_DRIVE_POS_GAINS.kD, kDT_DRIVE_POS_GAINS.kSlotID);
        m_rightPID.setIZone(kDT_DRIVE_POS_GAINS.kIzone, kDT_DRIVE_POS_GAINS.kSlotID);
        m_rightPID.setFF(kDT_DRIVE_POS_GAINS.kFF, kDT_DRIVE_POS_GAINS.kSlotID);
        m_rightPID.setOutputRange(kDT_DRIVE_POS_GAINS.kMinOutput, kDT_DRIVE_POS_GAINS.kMaxOutput, kDT_DRIVE_POS_GAINS.kSlotID);

        // TURN
        m_leftPID.setP(kDT_TURN_POS_GAINS.kP, kDT_TURN_POS_GAINS.kSlotID);
        m_leftPID.setI(kDT_TURN_POS_GAINS.kI, kDT_TURN_POS_GAINS.kSlotID);
        m_leftPID.setD(kDT_TURN_POS_GAINS.kD, kDT_TURN_POS_GAINS.kSlotID);
        m_leftPID.setIZone(kDT_TURN_POS_GAINS.kIzone, kDT_TURN_POS_GAINS.kSlotID);
        m_leftPID.setFF(kDT_TURN_POS_GAINS.kFF, kDT_TURN_POS_GAINS.kSlotID);
        m_leftPID.setOutputRange(kDT_TURN_POS_GAINS.kMinOutput, kDT_TURN_POS_GAINS.kMaxOutput, kDT_TURN_POS_GAINS.kSlotID);

        m_rightPID.setP(kDT_TURN_POS_GAINS.kP, kDT_TURN_POS_GAINS.kSlotID);
        m_rightPID.setI(kDT_TURN_POS_GAINS.kI, kDT_TURN_POS_GAINS.kSlotID);
        m_rightPID.setD(kDT_TURN_POS_GAINS.kD, kDT_TURN_POS_GAINS.kSlotID);
        m_rightPID.setIZone(kDT_TURN_POS_GAINS.kIzone, kDT_TURN_POS_GAINS.kSlotID);
        m_rightPID.setFF(kDT_TURN_POS_GAINS.kFF, kDT_TURN_POS_GAINS.kSlotID);
        m_rightPID.setOutputRange(kDT_TURN_POS_GAINS.kMinOutput, kDT_TURN_POS_GAINS.kMaxOutput, kDT_TURN_POS_GAINS.kSlotID);

        // Velocity PID

        // DRIVE
        m_leftPID.setP(kDT_DRIVE_VEL_GAINS.kP, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_leftPID.setI(kDT_DRIVE_VEL_GAINS.kI, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_leftPID.setD(kDT_DRIVE_VEL_GAINS.kD, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_leftPID.setIZone(kDT_DRIVE_VEL_GAINS.kIzone, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_leftPID.setFF(kDT_DRIVE_VEL_GAINS.kFF, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_leftPID.setOutputRange(kDT_DRIVE_VEL_GAINS.kMinOutput, kDT_DRIVE_VEL_GAINS.kMaxOutput, kDT_DRIVE_VEL_GAINS.kSlotID);

        m_rightPID.setP(kDT_DRIVE_VEL_GAINS.kP, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_rightPID.setI(kDT_DRIVE_VEL_GAINS.kI, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_rightPID.setD(kDT_DRIVE_VEL_GAINS.kD, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_rightPID.setIZone(kDT_DRIVE_VEL_GAINS.kIzone, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_rightPID.setFF(kDT_DRIVE_VEL_GAINS.kFF, kDT_DRIVE_VEL_GAINS.kSlotID);
        m_rightPID.setOutputRange(kDT_DRIVE_VEL_GAINS.kMinOutput, kDT_DRIVE_VEL_GAINS.kMaxOutput, kDT_DRIVE_VEL_GAINS.kSlotID);

        // TURN
        m_leftPID.setP(kDT_TURN_VEL_GAINS.kP, kDT_TURN_VEL_GAINS.kSlotID);
        m_leftPID.setI(kDT_TURN_VEL_GAINS.kI, kDT_TURN_VEL_GAINS.kSlotID);
        m_leftPID.setD(kDT_TURN_VEL_GAINS.kD, kDT_TURN_VEL_GAINS.kSlotID);
        m_leftPID.setIZone(kDT_TURN_VEL_GAINS.kIzone, kDT_TURN_VEL_GAINS.kSlotID);
        m_leftPID.setFF(kDT_TURN_VEL_GAINS.kFF, kDT_TURN_VEL_GAINS.kSlotID);
        m_leftPID.setOutputRange(kDT_TURN_VEL_GAINS.kMinOutput, kDT_TURN_VEL_GAINS.kMaxOutput, kDT_TURN_VEL_GAINS.kSlotID);

        m_rightPID.setP(kDT_TURN_VEL_GAINS.kP, kDT_TURN_VEL_GAINS.kSlotID);
        m_rightPID.setI(kDT_TURN_VEL_GAINS.kI, kDT_TURN_VEL_GAINS.kSlotID);
        m_rightPID.setD(kDT_TURN_VEL_GAINS.kD, kDT_TURN_VEL_GAINS.kSlotID);
        m_rightPID.setIZone(kDT_TURN_VEL_GAINS.kIzone, kDT_TURN_VEL_GAINS.kSlotID);
        m_rightPID.setFF(kDT_TURN_VEL_GAINS.kFF, kDT_TURN_VEL_GAINS.kSlotID);
        m_rightPID.setOutputRange(kDT_TURN_VEL_GAINS.kMinOutput, kDT_TURN_VEL_GAINS.kMaxOutput, kDT_TURN_VEL_GAINS.kSlotID);

        // Diff Drive
        m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        //m_gyro = new PigeonIMU(kGYRO_ID);

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
        m_diffDrive.tankDrive(deadzone(applyCurve(-leftValue, kDRIVE_THRESHOLD, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(-rightValue, kDRIVE_TURN_THRESHOLD, kDRIVE_CURVE_ODD_NUMBER), kTURN_DEADZONE), squareInput);
    }

    public void useArcadeDrive(double speedValue, double turnValue, boolean squareInput) {
        m_diffDrive.arcadeDrive(deadzone(applyCurve(-speedValue, kDRIVE_THRESHOLD, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(-turnValue, kDRIVE_TURN_THRESHOLD, kDRIVE_CURVE_ODD_NUMBER), kTURN_DEADZONE), squareInput);
    }

    /*
     *  O is the output to the motors
     *  I is the input for the joystick
     *  n is the odd number that determines how much curve is stretched toward the lower values
     *  T is the threshold after which the robot will start to move
     */
    public double applyCurve(double joystickInput, double threshold, int n) {

        if (joystickInput > 0) {

            return (((1 - threshold) * Math.pow(joystickInput, n)) + threshold);

        } else if (joystickInput < 0) {

            return (((1 - threshold) * Math.pow(joystickInput, n)) - threshold);

        }

        return 0;
    }

    public double deadzone(double value, double deadzone) {

        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;

    }

    public void setLeftSpeed(double speed) {
        m_leftLeader.set(speed);
    }

    public void setRightSpeed(double speed) {
        m_rightLeader.set(speed);
    }

    public void setSpeed(double speed) {
        m_leftLeader.set(speed);
        m_rightLeader.set(speed);
    }

    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> setSpeed(speed));
    }

    public void stop() {
        m_leftLeader.set(0.0);
        m_rightLeader.set(0.0);
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);
    }

    public double getLeftPosSetpoint() {
        return m_leftPosSetpoint;
    }

    public double getRightPosSetpoint() {
        return m_rightPosSetpoint;
    }

    public double getLeftPosSetpointIn() {
        return Units.metersToInches(getLeftPosSetpoint());
    }

    public double getRightPosSetpointIn() {
        return Units.metersToInches(getRightPosSetpoint());
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

    public double[] getLeftPosResponseIn() {
        m_leftPosResponseIn[0] = getLeftPosSetpointIn();
        m_leftPosResponseIn[1] = getLeftPosIn();
        return m_leftPosResponseIn;
    }

    public double[] getRightPosResponseIn() {
        m_rightPosResponseIn[0] = getRightPosSetpointIn();
        m_rightPosResponseIn[1] = getRightPosIn();
        return m_rightPosResponseIn;
    }

    public double getLeftVelSetpoint() {
        return m_leftVelSetpoint;
    }

    public double getRightVelSetpoint() {
        return m_rightVelSetpoint;
    }

    public double getLeftVelSetpointIn() {
        return Units.metersToInches(getLeftVelSetpoint());
    }

    public double getRightVelSetpointIn() {
        return Units.metersToInches(getRightVelSetpoint());
    }

    public double getLeftVel() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightVel() {
        return m_rightEncoder.getVelocity();
    }

    public double getLeftVelIn() {
        return Units.metersToInches(getLeftVel());
    }

    public double getRightVelIn() {
        return Units.metersToInches(getRightVel());
    }

    public double[] getLeftVelResponseIn() {
        m_leftVelResponseIn[0] = getLeftVelSetpointIn();
        m_leftVelResponseIn[1] = getLeftVelIn();
        return m_leftVelResponseIn;
    }

    public double[] getRightVelResponseIn() {
        m_rightVelResponseIn[0] = getRightVelSetpointIn();
        m_rightVelResponseIn[1] = getRightVelIn();
        return m_rightVelResponseIn;
    }

    public double getLeftVoltage() {
        return (m_leftLeader.getAppliedOutput() * m_leftLeader.getBusVoltage());
    }

    public double getLeftVoltage_bus() {
        return m_leftLeader.getBusVoltage();
    }
    public double getLeftVoltage_output() {
        return m_leftLeader.getAppliedOutput();
    }

    public double getRightVoltage() {
        return (m_rightLeader.getAppliedOutput() * m_rightLeader.getBusVoltage());
    }

    public double getRightVoltage_bus() {
        return m_rightLeader.getBusVoltage();
    }
    public double getRightVoltage_output() {
        return m_rightLeader.getAppliedOutput();
    }

    public double getLeftCurrent() {
        return m_leftLeader.getOutputCurrent();
    }

    public double getRightCurrent() {    
        return m_rightLeader.getOutputCurrent();
    }

    public double getLeftTemp() {
        return m_leftLeader.getMotorTemperature();
    }

    public double getRightTemp() {
        return m_rightLeader.getMotorTemperature();
    }

    public double getLeftFollowerTemp() {
        return m_leftFollower.getMotorTemperature();
    }

    public double getRightFollowerTemp() {
        return m_rightFollower.getMotorTemperature();
    }


    /*public double getRoll() {
        return m_gyro.getRoll();
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }*/

    public void updateGains(int slotID) {
        m_leftPID.setP(m_kP, slotID);
        m_leftPID.setI(m_kI, slotID);
        m_leftPID.setD(m_kD, slotID);
        m_leftPID.setFF(m_kFF, slotID);

        m_rightPID.setP(m_kP, slotID);
        m_rightPID.setI(m_kI, slotID);
        m_rightPID.setD(m_kD, slotID);
        m_rightPID.setFF(m_kFF, slotID);
    }

    public void updateNow() {
        m_updateNow = true;
    }

    public Command updateNowCommand() {
        return Commands.runOnce(() -> updateNow(), this);
    }

    public void tune() {

        m_leftPosSetpoint = 0;
        m_leftPosSetpointIn = 0;

        m_rightPosSetpoint = 0;
        m_rightPosSetpointIn = 0;

        m_leftVelSetpoint = 0;
        m_leftVelSetpointIn = 0;

        m_rightVelSetpoint = 0;
        m_rightVelSetpointIn = 0;

        m_leftPosResponseIn = new double[2];
        m_rightPosResponseIn = new double[2];

        m_leftVelResponseIn = new double[2];
        m_rightVelResponseIn = new double[2];

        // ONLY ENABLE ONE AT A TIME (ERROR WILL OCCUR WHEN MULTIPLE ARE TRUE AT THE SAME TIME)
        m_DRIVE_POS = true; 
        m_TURN_POS = false;
        m_DRIVE_VEL = false;
        m_TURN_VEL = false;

        if(m_DRIVE_POS) {open_DRIVE_POS();}
        if(m_TURN_POS) {open_TURN_POS();}
        if(m_DRIVE_VEL) {open_DRIVE_VEL();}
        if(m_TURN_VEL) {open_TURN_VEL();}
        
    }

    public void periodicTuning() {

        if(m_DRIVE_POS) {periodic_DRIVE_POS();}
        if(m_TURN_POS) {periodic_TURN_POS();}
        if(m_DRIVE_VEL) {periodic_DRIVE_VEL();}
        if(m_TURN_VEL) {periodic_TURN_VEL();}

    }

    public void open_DRIVE_POS() {

        m_tab = Shuffleboard.getTab("DT - PosDrive Tuner");

        m_kP = kDT_DRIVE_POS_GAINS.kP;
        m_kI = kDT_DRIVE_POS_GAINS.kI;
        m_kD = kDT_DRIVE_POS_GAINS.kD;
        m_kFF = kDT_DRIVE_POS_GAINS.kFF;

        e_kP = m_tab.add("Set kP", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Set kI", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Set kD", m_kD).withPosition(0, 2).getEntry();
        e_kFF = m_tab.add("Set kFF", m_kFF).withPosition(0, 3).getEntry();

        e_leftSetpointPosIn = m_tab.add("L.SetPos (In)", m_leftPosSetpointIn).withPosition(2, 0).getEntry();
        e_rightSetpointPosIn = m_tab.add("R.SetPos (In)", m_rightPosSetpointIn).withPosition(6, 0).getEntry();

        e_leftSetpointVelIn = m_tab.add("L.SetVel (In:sec)", m_leftVelSetpointIn).withPosition(4, 0).getEntry();
        e_rightSetpointVelIn = m_tab.add("R.SetVel (In:sec)", m_rightVelSetpointIn).withPosition(8, 0).getEntry();

        m_tab.addDoubleArray("L.Response (In)", this::getLeftPosResponseIn).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);
        m_tab.addDoubleArray("R.Response (In)", this::getRightPosResponseIn).withPosition(7,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("L.Volts Bus (V)", this::getLeftVoltage_bus).withPosition(2, 4);
        m_tab.addNumber("L.Volts Out (V)", this::getLeftVoltage_output).withPosition(2, 5);
        m_tab.addNumber("L.Volts (V)", this::getLeftVoltage).withPosition(2, 1);
        m_tab.addNumber("L.Amps (A)", this::getLeftCurrent).withPosition(2, 2);
        m_tab.addNumber("L.L.Temp (C)", this::getLeftTemp).withPosition(2, 3);
        m_tab.addNumber("L.Pos (in)", this::getLeftPosIn).withPosition(3, 0);
        m_tab.addNumber("L.Vel (in:sec)", this::getLeftVelIn).withPosition(5, 0);
        m_tab.addNumber("L.F.Temp (C)", this::getLeftFollowerTemp).withPosition(1, 3);

        // Mid
        /*m_tab.addNumber("Yaw (deg)", this::getYaw).withPosition(6, 1);
        m_tab.addNumber("Pitch (deg)", this::getPitch).withPosition(6, 2);
        m_tab.addNumber("Roll (deg)", this::getRoll).withPosition(6, 3);*/

        // Right Telemetry
        m_tab.addNumber("R.Volts Bus (V)", this::getRightVoltage_bus).withPosition(10, 4);
        m_tab.addNumber("R.Volts Out (V)", this::getRightVoltage_output).withPosition(10, 5);
        m_tab.addNumber("R.Volts (V)", this::getRightVoltage).withPosition(10, 1);
        m_tab.addNumber("R.Amps (A)", this::getRightCurrent).withPosition(10, 2);
        m_tab.addNumber("R.Temp (C)", this::getRightTemp).withPosition(10, 3);
        m_tab.addNumber("R.Pos (in)", this::getRightPosIn).withPosition(7, 0);
        m_tab.addNumber("R.Vel (in:sec)", this::getRightVelIn).withPosition(9, 0);
        m_tab.addNumber("R.F.Temp (C)", this::getRightFollowerTemp).withPosition(11, 3);

    }

    public void periodic_DRIVE_POS() {

        var kP = e_kP.getDouble(kDT_DRIVE_POS_GAINS.kP);
        var kI = e_kI.getDouble(kDT_DRIVE_POS_GAINS.kI);
        var kD = e_kD.getDouble(kDT_DRIVE_POS_GAINS.kD);
        var kFF = e_kFF.getDouble(kDT_DRIVE_POS_GAINS.kFF);

        if(m_updateNow) {

            if(kP != m_kP) {m_kP = kP;}
            if(kI != m_kI) {m_kI = kI;}
            if(kD != m_kD) {m_kD = kD;}
            if(kFF != m_kFF) {m_kFF = kFF;}
            
            updateGains(kDT_DRIVE_POS_GAINS.kSlotID);

            m_updateNow = false;
        }

    }

    public void open_TURN_POS() {

    }

    public void periodic_TURN_POS() {
        
    }

    public void open_DRIVE_VEL() {

        m_tab = Shuffleboard.getTab("DT - VelDrive Tuner");

        m_kP = kDT_DRIVE_VEL_GAINS.kP;
        m_kI = kDT_DRIVE_VEL_GAINS.kI;
        m_kD = kDT_DRIVE_VEL_GAINS.kD;
        m_kFF = kDT_DRIVE_VEL_GAINS.kFF;

        e_kP = m_tab.add("Set kP", m_kP).withPosition(0, 0).getEntry();
        e_kI = m_tab.add("Set kI", m_kI).withPosition(0, 1).getEntry();
        e_kD = m_tab.add("Set kD", m_kD).withPosition(0, 2).getEntry();
        e_kFF = m_tab.add("Set kFF", m_kFF).withPosition(0, 3).getEntry();

        e_leftSetpointPosIn = m_tab.add("L.Setpt (In)", m_leftPosSetpointIn).withPosition(2, 0).getEntry();
        e_rightSetpointPosIn = m_tab.add("R.Setpt (In)", m_rightPosSetpointIn).withPosition(6, 0).getEntry();

        e_leftSetpointVelIn = m_tab.add("L.Setpt (In:sec)", m_leftVelSetpointIn).withPosition(4, 0).getEntry();
        e_rightSetpointVelIn = m_tab.add("R.Setpt (In:sec)", m_rightVelSetpointIn).withPosition(8, 0).getEntry();

        m_tab.addDoubleArray("L.Response (In:sec)", this::getLeftVelResponseIn).withPosition(3,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);
        m_tab.addDoubleArray("R.Response (In:sec)", this::getRightVelResponseIn).withPosition(7,1).withSize(3,3).withWidget(BuiltInWidgets.kGraph);

        // Left Telemetry
        m_tab.addNumber("L.Volts (V)", this::getLeftVoltage).withPosition(2, 1);
        m_tab.addNumber("L.Amps (A)", this::getLeftCurrent).withPosition(2, 2);
        m_tab.addNumber("L.Temp (C)", this::getLeftTemp).withPosition(2, 3);
        m_tab.addNumber("L.Pos (in)", this::getLeftPosIn).withPosition(3, 0);
        m_tab.addNumber("L.Vel (in:sec)", this::getLeftVelIn).withPosition(5, 0);

        // Mid
        /*m_tab.addNumber("Yaw (deg)", this::getYaw).withPosition(6, 1);
        m_tab.addNumber("Pitch (deg)", this::getPitch).withPosition(6, 2);
        m_tab.addNumber("Roll (deg)", this::getRoll).withPosition(6, 3);*/

        // Right Telemetry
        m_tab.addNumber("R.Volts (V)", this::getRightVoltage).withPosition(10, 1);
        m_tab.addNumber("R.Amps (A)", this::getRightCurrent).withPosition(10, 2);
        m_tab.addNumber("R.Temp (C)", this::getRightTemp).withPosition(10, 3);
        m_tab.addNumber("R.Pos (in)", this::getRightPosIn).withPosition(7, 0);
        m_tab.addNumber("R.Vel (in:sec)", this::getRightVelIn).withPosition(9, 0);

    }

    public void periodic_DRIVE_VEL() {

        var kP = e_kP.getDouble(kDT_DRIVE_VEL_GAINS.kP);
        var kI = e_kI.getDouble(kDT_DRIVE_VEL_GAINS.kI);
        var kD = e_kD.getDouble(kDT_DRIVE_VEL_GAINS.kD);
        var kFF = e_kFF.getDouble(kDT_DRIVE_VEL_GAINS.kFF);

        if(m_updateNow) {

            if(kP != m_kP) {m_kP = kP;}
            if(kI != m_kI) {m_kI = kI;}
            if(kD != m_kD) {m_kD = kD;}
            if(kFF != m_kFF) {m_kFF = kFF;}
            
            updateGains(kDT_DRIVE_VEL_GAINS.kSlotID);

            m_updateNow = false;
        }
        
    }

    public void open_TURN_VEL() {

    }

    public void periodic_TURN_VEL() {
        
    }
    
}