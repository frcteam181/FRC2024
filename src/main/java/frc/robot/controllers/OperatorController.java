package frc.robot.controllers;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm_actions.moveArmDown;
import frc.robot.commands.arm_actions.moveArmUp;
import frc.robot.commands.defaults.DefaultArm;
import frc.robot.commands.defaults.DefaultFlyWheel;
import frc.robot.commands.defaults.DefaultIntake;
import frc.robot.commands.defaults.DefaultWrist;
import frc.robot.commands.wrist_actions.tiltWristDown;
import frc.robot.commands.wrist_actions.tiltWristUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class OperatorController {

    private Arm m_arm;
    private Intake m_intake;
    private Wrist m_wrist;
    private FlyWheel m_flyWheel;

    private JoystickButton m_a, m_y, m_lb, m_rb;

    private XboxController m_controller;

    public OperatorController() {

        m_controller = new XboxController(kOPERATOR);

        m_arm = kARM;
        m_intake = kINTAKE;
        m_wrist = kWRIST;
        m_flyWheel = kFLYWHEEL;

        m_a = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_y = new JoystickButton(m_controller, XboxController.Button.kY.value);
        m_lb = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
        m_rb = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);

        m_arm.setDefaultCommand(new DefaultArm(m_controller));
        m_intake.setDefaultCommand(new DefaultIntake(m_controller));
        m_wrist.setDefaultCommand(new DefaultWrist(m_controller));
        m_flyWheel.setDefaultCommand(new DefaultFlyWheel(m_controller));

        bindButtons();

    }

    public void bindButtons() {

        m_a.whileTrue(new moveArmDown());
        m_y.whileTrue(new moveArmUp());

        m_lb.whileTrue(new tiltWristDown());
        m_rb.whileTrue(new tiltWristUp());

    }
    
}
