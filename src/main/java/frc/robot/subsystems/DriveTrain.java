package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;

    private RelativeEncoder m_leftEncoder, m_rightEncoder;

    private SparkPIDController m_leftPID, m_rightPID;

    private DifferentialDrive m_diffDrive;

    private boolean m_isTuning;

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
        m_rightLeader.setIdleMode(IdleMode.kBrake);

        m_leftEncoder = m_leftLeader.getEncoder();
        m_leftEncoder.setInverted(false);
        m_leftEncoder.setPositionConversionFactor(kDRIVETRAIN_POS_FACTOR_METER); // m
        m_leftEncoder.setVelocityConversionFactor(kDRIVETRAIN_VEL_FACTOR_METER); // m/sec

        m_rightEncoder = m_rightLeader.getEncoder();
        m_rightEncoder.setInverted(false);
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

    // To default the values to use direct joystick values, change n = 1; and kDRIVE_DEADZONE to 0;
    public void useTankDrive(double leftValue, double rightValue, boolean squareInput) {
        m_diffDrive.tankDrive(deadzone(applyCurve(leftValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(rightValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), squareInput);
    }

    public void useArcadeDrive(double speedValue, double turnValue, boolean squareInput) {
        m_diffDrive.arcadeDrive(deadzone(applyCurve(speedValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), deadzone(applyCurve(turnValue, kDRIVE_CURVE_ODD_NUMBER), kDRIVE_DEADZONE), squareInput);
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

    public void tune() {
        
    }
    
}