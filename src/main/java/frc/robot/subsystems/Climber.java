package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private RelativeEncoder m_leftEncoder, m_rightEncoder;
    private DigitalInput m_leftHomeSwitch, m_rightHomeSwitch;
    private boolean m_isTuning, m_isLeftHome, m_isRightHome;

    // Tuner
    private ShuffleboardTab m_tab;

    public Climber(boolean isTuning) {

        m_leftMotor = new CANSparkMax(kLEFT_CLIMBER_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(kRIGHT_CLIMBER_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setIdleMode(IdleMode.kCoast);

        //m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        m_leftMotor.follow(m_rightMotor, true);

        m_leftHomeSwitch = new DigitalInput(kLEFT_CLIMBER_HOME_SWITCH_CHANNEL);
        m_rightHomeSwitch = new DigitalInput(kRIGHT_CLIMBER_HOME_SWITCH_CHANNEL);

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {
        getHomeStatus();
        /* TUNING */
        if(m_isTuning) {periodicTuning();}
    }

    public void resetLeftEncoder() {
        m_leftEncoder.setPosition(0.0);
    }

    public void resetRightEncoder() {
        m_rightEncoder.setPosition(0.0);
    }

    public double getLeftPos() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPos() {
        return m_rightEncoder.getPosition();
    }

    public void moveLeftClimber(double leftSpeed) {
        if(m_isLeftHome) {m_leftMotor.set(0.0);}
        else {m_leftMotor.set(leftSpeed);}
    } 

    public void moveRightClimber(double rightSpeed) {
        if(m_isRightHome) {m_rightMotor.set(0.0);}
        else {m_rightMotor.set(rightSpeed);}
    }

    public void moveClimber(double speed) { //Follower mode only
        moveRightClimber(speed); //Leader
    }

    public void moveClimber(double leftSpeed, double rightSpeed) { // Not follower mode
        moveLeftClimber(leftSpeed);
        moveRightClimber(rightSpeed);
    }

    public void enableBreaks() {
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
    }

    public boolean isLeftHome() {
        return m_isLeftHome;
    }

    public boolean isRightHome() {
        return m_isRightHome;
    }

    public void getHomeStatus() {
        m_isLeftHome = m_leftHomeSwitch.get();
        m_isRightHome = m_rightHomeSwitch.get();
    }

    // Tuning

    public void tune() {

        m_tab = Shuffleboard.getTab("Climber Tuner");

        m_tab.addNumber("LeftPos", this::getLeftPos).withPosition(0, 0);
        m_tab.addNumber("RightPos", this::getRightPos).withPosition(1, 0);

        m_tab.addBoolean("isLeftHome", this::isLeftHome).withPosition(0, 1).withSize(1, 1);
        m_tab.addBoolean("isRightHome", this::isRightHome).withPosition(1, 1).withSize(1, 1);

    }

    public void periodicTuning() {

    }
    
}
