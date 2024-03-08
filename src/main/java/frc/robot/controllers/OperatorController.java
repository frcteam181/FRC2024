package frc.robot.controllers;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm_actions.moveArmDown;
import frc.robot.commands.arm_actions.moveArmUp;
import frc.robot.commands.defaults.DefaultArm;
import frc.robot.commands.defaults.DefaultFlyWheel;
import frc.robot.commands.defaults.DefaultIntake;
import frc.robot.commands.defaults.DefaultWrist;
import frc.robot.commands.flywheel_actions.toggleFlywheel;
import frc.robot.commands.intake_actions.feedFlywheel;
import frc.robot.commands.intake_actions.teleIntake;
import frc.robot.commands.a_presets.*;
import frc.robot.commands.presets.back_amp_preset;
import frc.robot.commands.presets.back_high_speaker_preset;
import frc.robot.commands.presets.back_low_speaker_preset;
import frc.robot.commands.presets.front_amp_preset;
import frc.robot.commands.presets.front_high_speaker;
import frc.robot.commands.presets.intake_preset;
import frc.robot.commands.presets.stow_away_preset;
import frc.robot.commands.wrist_actions.tiltWristDown;
import frc.robot.commands.wrist_actions.tiltWristUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class OperatorController {

    private Arm m_arm;
    private Intake m_intake;
    private Wrist m_wrist;
    private Flywheel m_flyWheel;

    // Xbox Controller
    private XboxController m_controllerXbox;
    private JoystickButton m_a, m_y, m_x, m_b, m_lb, m_rb, m_start;
    private POVButton m_up, m_dw, m_l, m_r;

    // PS4 Controller
    private PS4Controller m_controllerPS4;
    private JoystickButton m_xPS4, m_sPS4, m_cPS4, m_tPS4, m_lbPS4, m_rbPS4, m_optionsPS4, m_sharePS4;
    private POVButton m_upPS4, m_dwPS4, m_lPS4, m_rPS4;

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

        //m_arm.setDefaultCommand(new DefaultArm(m_controllerXbox, m_controllerPS4, isPS4));
        m_intake.setDefaultCommand(new DefaultIntake(m_controllerXbox, m_controllerPS4, isPS4));
        //m_wrist.setDefaultCommand(new DefaultWrist(m_controllerXbox, m_controllerPS4, isPS4));
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

        m_upPS4 = new POVButton(m_controllerPS4, 0);
        m_rPS4 = new POVButton(m_controllerPS4, 90);
        m_dwPS4 = new POVButton(m_controllerPS4, 180);
        m_lPS4 = new POVButton(m_controllerPS4, 270);

        bindPS4Buttons();

    }

    public void bindPS4Buttons() {
        
        // COMPETITION (DO NOT CHANGE)
        m_tPS4.whileTrue(new feedFlywheel());                       // Feed Flywheels
        //m_rbPS4.and(m_tPS4).whileTrue(new feedFlywheel());                       // Feed Flywheels
        m_xPS4.onTrue(new toggleFlywheel());                        // Start Flywheels
        //m_rbPS4.and(m_xPS4).onTrue(new toggleFlywheel());                        // Start Flywheels
        //m_xPS4.onTrue(new toggleFlywheel());

        m_sPS4.onTrue(new front_amp_preset());                      // Front Amp
        m_cPS4.onTrue(new back_amp_preset());                       // Back Amp

        m_upPS4.onTrue(new front_high_speaker());                     // Front (HIGH) Speaker (Only have one option for front)
        m_dwPS4.onTrue(new back_low_speaker_preset());                        // Back Low Speaker

        m_lPS4.onTrue(new intake_preset());              // Intake
        m_rPS4.onTrue(new stow_away_preset());               // Stow Away

        m_lbPS4.and(m_upPS4).onTrue(new front_high_speaker());           // Front (HIGH) Speaker (Only have one option for front)
        m_lbPS4.and(m_dwPS4).onTrue(new back_high_speaker_preset());           // Back High Speaker

        m_rbPS4.whileTrue(new teleIntake());

        // TESTING & OTHERS
        //m_lbPS4.onTrue(m_intake.setVelCommand(30000)).onFalse(m_intake.setVelCommand(0));
        //m_rbPS4.onTrue(m_flyWheel.setVelCommand(30000)).onFalse(m_flyWheel.setVelCommand(0));

        //m_sPS4.onTrue(m_wrist.setGoalCommand(Math.toRadians(-100)));

        //m_rbPS4.onTrue(m_arm.setGoalCommand(Math.toRadians(45)));
        //m_lbPS4.onTrue(m_arm.setGoalCommand(Math.toRadians(0)));

        //m_cPS4.onTrue(new intake_preset());

        //m_sPS4.onTrue(m_arm.setUserGoalCommand());

        //m_sharePS4.whileTrue(new tiltWristDown());
        //m_optionsPS4.whileTrue(new tiltWristUp());

        //m_sPS4.onTrue(m_intake.setVelCommand(5000.0));
        //m_xPS4.onTrue(m_intake.setVelCommand(0.0));

        //m_optionsPS4.onTrue(m_arm.updateNowCommand()); //arm
        //m_optionsPS4.onTrue(m_wrist.updateNowCommand()); //wrist
        //m_optionsPS4.onTrue(m_intake.updateNowCommand()); //intake
        //m_optionsPS4.onTrue(m_intake.updateNowCommand()); //Flywheel

        //m_sPS4.onTrue(m_intake.setTuningVelCommand());
        //m_xPS4.onTrue(m_intake.setVelCommand(Math.toRadians(0)));

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

        m_up = new POVButton(m_controllerXbox, 0);
        m_r = new POVButton(m_controllerXbox, 90);
        m_dw = new POVButton(m_controllerXbox, 180);
        m_l = new POVButton(m_controllerXbox, 270);

        bindXboxButtons();

    }

    public void bindXboxButtons() {

         // COMPETITION (DO NOT CHANGE)
        //m_y.whileTrue(new feedFlywheel());                       // Feed Flywheels
        //m_rbPS4.and(m_tPS4).whileTrue(new feedFlywheel());                       // Feed Flywheels
        //m_a.onTrue(new toggleFlywheel());                        // Start Flywheels
        //m_rbPS4.and(m_xPS4).onTrue(new toggleFlywheel());                        // Start Flywheels
        //m_xPS4.onTrue(new toggleFlywheel());

        m_x.onTrue(new a_front_amp_preset());                      // Front Amp
        m_b.onTrue(new a_back_amp_preset());                       // Back Amp

        m_up.onTrue(new a_front_high_speaker());                     // Front (HIGH) Speaker (Only have one option for front)
        m_dw.onTrue(new a_back_low_speaker_preset());                        // Back Low Speaker

        m_l.onTrue(new a_intake_preset());              // Intake
        m_r.onTrue(new a_stow_away_preset());               // Stow Away

        m_lb.and(m_up).onTrue(new a_front_high_speaker());           // Front (HIGH) Speaker (Only have one option for front)
        //m_lb.and(m_dw).onTrue(new back_high_speaker_preset());           // Back High Speaker
        m_lb.and(m_dw).onTrue(new a_back_high_speaker_preset());

        //m_rb.whileTrue(new teleIntake());
        //m_a.onTrue(new toggleFlywheel());
        //m_y.whileTrue(new feedFlywheel());

        //m_x.onTrue(m_arm.updateNowCommand());

        //m_a.whileTrue(new tiltWristDown());
        //m_y.whileTrue(new tiltWristUp());

    }

    public void rumble() {
        m_controllerPS4.setRumble(null, kARM_GEAR_RATIO);
    }
    
}
