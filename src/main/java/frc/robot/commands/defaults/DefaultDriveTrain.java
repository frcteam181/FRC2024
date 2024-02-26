package frc.robot.commands.defaults;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DefaultDriveTrain extends Command {

    private DriveTrain m_driveTrain;

    private Joystick m_leftStick, m_rightStick, m_joystick;

    private XboxController m_controllerXbox;
    private PS4Controller m_controllerPS4;

    private boolean m_isTankDrive, m_isPS4, m_isJoystick, m_isSquared;
    private double m_leftYValue, m_rightYValue, m_forwardValue, m_turnValue;

    // Two Stick (Joystick)
    public DefaultDriveTrain(Joystick leftStick, Joystick rightStick, boolean squareInput) {

        m_leftStick = leftStick;
        m_rightStick = rightStick;

        m_isTankDrive = true;
        m_isJoystick = true;

        m_isSquared = squareInput;

        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_driveTrain);

    }

    // One Stick (Joystick)
    public DefaultDriveTrain(Joystick joystick, boolean squareInput) {

        m_joystick = joystick;

        m_isTankDrive = false;
        m_isJoystick = true;

        m_isSquared = squareInput;

        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_driveTrain);

    }

    // Two Stick (Controller)
    public DefaultDriveTrain(boolean isPS4, boolean isTankDrive, boolean squareInput) {

        m_isPS4 = isPS4;

        if(isPS4) {
            m_controllerPS4 = new PS4Controller(kCONTROLLER);
        } else {
            m_controllerXbox = new XboxController(kCONTROLLER);
        }

        m_isTankDrive = isTankDrive;
        m_isJoystick = false;

        m_isSquared = squareInput;

        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_driveTrain);

    }

    @Override
    public void execute() {

        var sign = 0;
        if(m_isPS4) {sign = 1;} 
        else {sign = -1;}

        getValues();

        // We use negative sign here because Y-axis on controllers and joysticks are flipped (Except PS4 Controllers)
        if (m_isTankDrive) {
            m_driveTrain.useTankDrive(sign * m_leftYValue, sign * m_rightYValue, m_isSquared);
        } else {
            m_driveTrain.useArcadeDrive(sign * m_forwardValue, sign * m_turnValue, m_isSquared);
        }
        
    }

    private void getValues() {

        if (m_isJoystick) {

            if (m_isTankDrive) {

                m_leftYValue = m_leftStick.getY();
                m_rightYValue = m_rightStick.getY();

            } else {

                m_forwardValue = m_joystick.getY();
                m_turnValue = m_joystick.getZ();

            }

        } else {

            if(m_isPS4) {
                if (m_isTankDrive) {

                m_leftYValue = m_controllerPS4.getLeftY();
                m_rightYValue = m_controllerPS4.getRightY();

                } else {

                    m_forwardValue = m_controllerPS4.getLeftY();
                    m_turnValue = m_controllerPS4.getRightX();
                    
                }
            } else {
                if (m_isTankDrive) {

                m_leftYValue = m_controllerXbox.getLeftY();
                m_rightYValue = m_controllerXbox.getRightY();

                } else {

                m_forwardValue = m_controllerXbox.getLeftY();
                m_turnValue = m_controllerXbox.getRightX();
                
                }
            }

        }

    }
    
}   
