package frc.robot.commands.defaults;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DefaultDriveTrain extends Command {

    private DriveTrain m_driveTrain;

    private Joystick m_leftStick, m_rightStick, m_joystick;

    private XboxController m_controller;

    private boolean m_isTankDrive, m_isJoystick, m_isSquared;
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
    public DefaultDriveTrain(XboxController controller, boolean isTankDrive, boolean squareInput) {

        m_controller = controller;

        m_isTankDrive = isTankDrive;
        m_isJoystick = false;

        m_isSquared = squareInput;

        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_driveTrain);

    }

    @Override
    public void execute() {

        getValues();

        // We use negative sign here because Y-axis on controllers and joysticks are flipped
        if (m_isTankDrive) {
            m_driveTrain.useTankDrive(-m_leftYValue, -m_rightYValue, m_isSquared);
        } else {
            m_driveTrain.useArcadeDrive(-m_forwardValue, -m_turnValue, m_isSquared);
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

            if (m_isTankDrive) {

                m_leftYValue = m_controller.getLeftY();
                m_rightYValue = m_controller.getRightY();

            } else {

                m_forwardValue = m_controller.getLeftY();
                m_turnValue = m_controller.getRightX();
                
            }

        }

    }
    
}   
