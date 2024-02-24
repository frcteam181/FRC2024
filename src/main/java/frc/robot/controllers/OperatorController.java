package frc.robot.controllers;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.PS4Controller;
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

    // Xbox Controller
    private XboxController m_controllerXbox;
    private JoystickButton m_a, m_y, m_x, m_b, m_lb, m_rb, m_start;

    // PS4 Controller
    private PS4Controller m_controllerPS4;
    private JoystickButton m_xPS4, m_sPS4, m_cPS4, m_tPS4, m_lbPS4, m_rbPS4, m_optionsPS4, m_sharePS4;

    public OperatorController(boolean isPS4) {

        m_arm = kARM;
        m_intake = kINTAKE;
        m_wrist = kWRIST;
        m_flyWheel = kFLYWHEEL;

        if (isPS4) {
            setUpPS4Controller();
        } else {
            setUpXboxController();
        }

        m_arm.setDefaultCommand(new DefaultArm(m_controllerXbox, m_controllerPS4, isPS4));
        m_intake.setDefaultCommand(new DefaultIntake(m_controllerXbox, m_controllerPS4, isPS4));
        m_wrist.setDefaultCommand(new DefaultWrist(m_controllerXbox, m_controllerPS4, isPS4));
        m_flyWheel.setDefaultCommand(new DefaultFlyWheel(m_controllerXbox, m_controllerPS4, isPS4));

    }

    public void setUpPS4Controller() {

        m_controllerPS4 = new PS4Controller(kOPERATOR);

        m_xPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kCross.value);
        m_sPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kSquare.value);
        m_cPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kCircle.value);
        m_tPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kTriangle.value);
        m_lbPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kL1.value);
        m_rbPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kR1.value);
        m_optionsPS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kOptions.value);
        m_sharePS4 = new JoystickButton(m_controllerPS4, PS4Controller.Button.kShare.value);

        bindPS4Buttons();

    }

    public void bindPS4Buttons() {
        
        //m_cPS4.onTrue(new moveArmDown());
        //m_xPS4.onTrue(new moveArmUp());

        m_cPS4.onTrue(m_wrist.setGoalCommand(Math.toRadians(50)));
        m_tPS4.onTrue(m_wrist.setGoalCommand(Math.toRadians(0)));
        //m_sPS4.onTrue(m_wrist.setGoalCommand(Math.toRadians(-100)));

        m_rbPS4.onTrue(m_arm.setGoalCommand(Math.toRadians(85)));
        m_lbPS4.onTrue(m_arm.setGoalCommand(Math.toRadians(0)));

        //m_sharePS4.whileTrue(new tiltWristDown());
        //m_xPS4.whileTrue(new tiltWristUp());

        //m_optionsPS4.onTrue(m_arm.updateNowCommand()); //arm
        //m_optionsPS4.onTrue(m_wrist.updateNowCommand()); //wrist
        m_optionsPS4.onTrue(m_intake.updateNowCommand()); //intake

        m_sPS4.onTrue(m_intake.setTuningVelCommand());
        m_xPS4.onTrue(m_intake.setVelCommand(Math.toRadians(0)));

    }

    public void setUpXboxController() {

        m_controllerXbox = new XboxController(kOPERATOR);

        m_a = new JoystickButton(m_controllerXbox, XboxController.Button.kA.value);
        m_y = new JoystickButton(m_controllerXbox, XboxController.Button.kY.value);
        m_x = new JoystickButton(m_controllerXbox, XboxController.Button.kX.value);
        m_b = new JoystickButton(m_controllerXbox, XboxController.Button.kB.value);
        m_lb = new JoystickButton(m_controllerXbox, XboxController.Button.kLeftBumper.value);
        m_rb = new JoystickButton(m_controllerXbox, XboxController.Button.kRightBumper.value);
        m_start = new JoystickButton(m_controllerXbox, XboxController.Button.kStart.value);

        bindXboxButtons();

    }

    public void bindXboxButtons() {

        m_x.whileTrue(new moveArmDown());
        m_y.whileTrue(new moveArmUp());
        m_b.onTrue(m_arm.goToCommand(Math.toRadians(90)));
        m_a.onTrue(m_arm.goToCommand(Math.toRadians(0)));

        m_lb.whileTrue(new tiltWristDown());
        m_rb.whileTrue(new tiltWristUp());

    }
    
}
