package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_armProfile;

    private boolean m_enabled, m_isTuning;
    private double m_period;

    public Arm(boolean isTuning) {

        m_leftMotor = new CANSparkMax(kLEFT_ARM_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(kRIGHT_ARM_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_encoder = m_rightMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kARM_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kARM_VEL_FACTOR_RAD); // rad/sec

        m_pidController = m_rightMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);

        m_leftMotor.setSmartCurrentLimit(kLEFT_ARM_CURRENT_LIMIT);
        m_rightMotor.setSmartCurrentLimit(kRIGHT_ARM_CURRENT_LIMIT);

        m_leftMotor.follow(m_rightMotor, true);

        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setIdleMode(IdleMode.kBrake);

        /* Trapezoid Profile */

        m_constraints = new TrapezoidProfile.Constraints(kMAX_ARM_VEL_RAD, kMAX_ARM_ACC_RAD); // rad/s & rad/s^2

        m_armProfile = new TrapezoidProfile(m_constraints);

        m_start = new TrapezoidProfile.State(0, 0);
        m_state = new TrapezoidProfile.State(0, 0);
        m_goal = new TrapezoidProfile.State(0, 0);

        m_enabled = false;

        m_period = 0.02;

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {

    
    }

    public void useState(TrapezoidProfile.State setpoint) {
        
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public void setGoal(double pos) {
        m_goal = new TrapezoidProfile.State(pos, 0);
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
        m_rightMotor.set(0);
    }

    public void tune() {

    }
    
}
