package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class FlyWheel extends SubsystemBase {

    private CANSparkMax m_leftFlyWheel, m_rightFlyWheel;

    private boolean m_isTuning;

    public FlyWheel(boolean isTuning) {

        m_leftFlyWheel = new CANSparkMax(kLEFT_FLYWHEEL_ID, MotorType.kBrushless);
        m_rightFlyWheel = new CANSparkMax(kRIGHT_FLYWHEEL_ID, MotorType.kBrushless);

        m_leftFlyWheel.restoreFactoryDefaults();
        m_rightFlyWheel.restoreFactoryDefaults();

        m_leftFlyWheel.follow(m_rightFlyWheel, true);

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
