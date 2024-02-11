package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.defaults.DefaultDriveTrain;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriverController {

    private DriveTrain m_driveTrain;

    private Joystick m_joystickOne, m_joystickTwo;
    private XboxController m_controller;

    public DriverController(boolean isJoystick, boolean isTankDrive, boolean isSquaredInput) {

        m_driveTrain = kDRIVE_TRAIN;

        if (isJoystick) {
            if (isTankDrive) {
                stickTankSetup(isSquaredInput);
            } else {
                stickArcadeSetup(isSquaredInput);
            }
        } else {
            controllerSetup(isTankDrive, isSquaredInput);
        }

    }

    private void stickTankSetup(boolean isSquaredInput) {

        m_joystickOne = new Joystick(kJOYSTICK_ONE);
        m_joystickTwo = new Joystick(kJOYSTICK_TWO);

        m_driveTrain.setDefaultCommand(new DefaultDriveTrain(m_joystickOne, m_joystickTwo, isSquaredInput));

    }

    private void stickArcadeSetup(boolean isSquaredInput) {

        m_joystickOne = new Joystick(kJOYSTICK_ONE);

        m_driveTrain.setDefaultCommand(new DefaultDriveTrain(m_joystickOne, isSquaredInput));

    }

    private void controllerSetup(boolean isTankDrive, boolean isSquaredInput) {

        m_controller = new XboxController(kCONTROLLER);

        m_driveTrain.setDefaultCommand(new DefaultDriveTrain(m_controller, isTankDrive, isSquaredInput));

    }
    
}
