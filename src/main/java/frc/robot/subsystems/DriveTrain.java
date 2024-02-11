package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;

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

        m_leftLeader.setSmartCurrentLimit(kLEFT_ARM_CURRENT_LIMIT);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        m_leftLeader.setInverted(true);
        m_rightLeader.setInverted(false);

        m_leftLeader.setIdleMode(IdleMode.kBrake);
        m_rightLeader.setIdleMode(IdleMode.kBrake);

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