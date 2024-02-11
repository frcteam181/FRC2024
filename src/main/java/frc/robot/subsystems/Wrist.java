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

public class Wrist extends SubsystemBase {

    private CANSparkMax m_leftWrist, m_rightWrist;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_wristProfile;

    private boolean m_enabled, m_isTuning;

    private double m_period;

    public Wrist(boolean isTuning) {

        m_leftWrist = new CANSparkMax(kLEFT_WRIST_ID, MotorType.kBrushless);
        m_rightWrist = new CANSparkMax(kRIGHT_WRIST_ID, MotorType.kBrushless);

        m_leftWrist.restoreFactoryDefaults();
        m_rightWrist.restoreFactoryDefaults();

        m_encoder = m_rightWrist.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kWRIST_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kWRIST_VEL_FACTOR_RAD); // rad/sec

        m_pidController = m_rightWrist.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);

        m_leftWrist.setSmartCurrentLimit(kLEFT_WRIST_CURRENT_LIMIT);
        m_rightWrist.setSmartCurrentLimit(kRIGHT_WRIST_CURRENT_LIMIT);

        m_leftWrist.follow(m_rightWrist, true);

        m_rightWrist.setIdleMode(IdleMode.kBrake);
        m_leftWrist.setIdleMode(IdleMode.kBrake);

        /* Trapezoid Profile */

        m_constraints = new TrapezoidProfile.Constraints(kMAX_WRIST_VEL_RAD, kMAX_WRIST_ACC_RAD); // rad/s & rad/s^2

        m_wristProfile = new TrapezoidProfile(m_constraints);

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


        m_wristProfile.calculate(m_period, m_state, m_goal);
        
    }

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
        m_rightWrist.set(0);
    }

    public void tune() {
        
    }
    
}
