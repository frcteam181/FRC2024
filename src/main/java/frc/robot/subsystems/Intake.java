package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {

    private CANSparkMax m_intake_motor;

    private RelativeEncoder m_encoder;

    private SparkPIDController m_pidController;

    private boolean m_isTuning, m_hasNote;

    private DigitalInput m_noteBeam;

    public Intake(boolean isTuning) {

        m_intake_motor = new CANSparkMax(kINTAKE_ID, MotorType.kBrushless);

        m_intake_motor.restoreFactoryDefaults();

        m_intake_motor.setSmartCurrentLimit(kINTAKE_CURRENT_LIMIT);

        m_intake_motor.setInverted(true);

        m_encoder = m_intake_motor.getEncoder();
        //m_encoder.setInverted(false);
        m_encoder.setPositionConversionFactor(kINTAKE_POS_FACTOR_RAD); // rad
        m_encoder.setVelocityConversionFactor(kINTAKE_VEL_FACTOR_RAD); // rad/sec

        m_pidController = m_intake_motor.getPIDController();

        m_pidController.setP(kINTAKE_GAINS.kP, kINTAKE_PID_SLOT_ID);
        m_pidController.setI(kINTAKE_GAINS.kI, kINTAKE_PID_SLOT_ID);
        m_pidController.setD(kINTAKE_GAINS.kD, kINTAKE_PID_SLOT_ID);
        m_pidController.setIZone(kINTAKE_GAINS.kIzone, kINTAKE_PID_SLOT_ID);
        m_pidController.setFF(kINTAKE_GAINS.kFF, kINTAKE_PID_SLOT_ID);
        m_pidController.setOutputRange(kINTAKE_GAINS.kMinOutput, kINTAKE_GAINS.kMaxOutput, kINTAKE_PID_SLOT_ID);

        //m_noteBeam = new DigitalInput(kNOTE_BEAM_CHANNEL);
        //m_hasNote = m_noteBeam.get();

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}
        
    }

    @Override
    public void periodic() {
        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    public void handle_intake(double speed) {
        m_intake_motor.set(speed);
    }

    public void periodicTuning() {

    }

    public void tune() {

    }
    
}
