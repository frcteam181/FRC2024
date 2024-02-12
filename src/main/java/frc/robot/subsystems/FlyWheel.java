package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class FlyWheel extends SubsystemBase {

    private CANSparkMax m_leftFlyWheel, m_rightFlyWheel;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_encoder;

    private boolean m_isTuning;

    public FlyWheel(boolean isTuning) {

        m_leftFlyWheel = new CANSparkMax(kLEFT_FLYWHEEL_ID, MotorType.kBrushless);
        m_rightFlyWheel = new CANSparkMax(kRIGHT_FLYWHEEL_ID, MotorType.kBrushless);

        m_leftFlyWheel.restoreFactoryDefaults();
        m_rightFlyWheel.restoreFactoryDefaults();

        m_leftFlyWheel.follow(m_rightFlyWheel, true);

        m_encoder = m_rightFlyWheel.getEncoder();
        m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kFLYWHEEL_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kFLYWHEEL_VEL_FACTOR_RAD); // rad/sec

        m_pidController = m_rightFlyWheel.getPIDController();

        m_pidController.setP(kFLYWHEEL_GAINS.kP, kFLYWHEEL_PID_SLOT_ID);
        m_pidController.setI(kFLYWHEEL_GAINS.kI, kFLYWHEEL_PID_SLOT_ID);
        m_pidController.setD(kFLYWHEEL_GAINS.kD, kFLYWHEEL_PID_SLOT_ID);
        m_pidController.setIZone(kFLYWHEEL_GAINS.kIzone, kFLYWHEEL_PID_SLOT_ID);
        m_pidController.setFF(kFLYWHEEL_GAINS.kFF, kFLYWHEEL_PID_SLOT_ID);
        m_pidController.setOutputRange(kFLYWHEEL_GAINS.kMinOutput, kFLYWHEEL_GAINS.kMaxOutput, kFLYWHEEL_PID_SLOT_ID);

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    public void shoot(double speed) {
        m_rightFlyWheel.set(speed);
    }

    public void tune() {
        
    }
    
}
