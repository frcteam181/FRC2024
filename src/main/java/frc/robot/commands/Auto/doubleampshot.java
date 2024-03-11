package frc.robot.commands.Auto;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class doubleampshot extends Command {
    private int curState;
    private Arm m_arm;
    private Wrist m_wrist;
    private Flywheel m_flywheel;
    private Intake m_intake;
    private Timer m_timer;
    private DriveTrain m_driveTrain;

    private boolean m_setWrist;
    private boolean m_setArm;
    public doubleampshot() {
        m_driveTrain = kDRIVE_TRAIN;
        m_arm = kARM;
        m_wrist = kWRIST;
        m_flywheel = kFLYWHEEL;
        m_intake = kINTAKE;
        m_timer = new Timer();
        curState = 0;
        m_setWrist = false;
        m_setArm = false;
        addRequirements(m_flywheel, m_intake);

    }

    //runs once

    /*
     * current state cheat sheet:
     * 
     * 0 = arm and wrist need to go to first position
     * 1 = note shot, moving to intake position
     * 2 = intaking, moving forward & waiting to pick up
     * 3 = note detected, moving back
     * 4 = moving back to scoring position
     * 5 = score & stow away
     * 
     */

    @Override
    public void initialize() {
        System.out.println("AWESOME AMP! Pre Load & Taxi Started!");

        //m_arm.setGoal(kBACK_AMP_PRESET.kArmPos);

        m_timer.reset();
        m_timer.stop();

       // m_flywheel.setSpeed(1.0); flywheel off ;)

    }

    //runs every cycle

    @Override
    public void execute() {

        if(curState == 0){
            preloadscore();
        }
        if(curState == 1){
            intakePosition();
        }

        if(curState == 2){
            groundPickup();
        }

        if(curState == 3){
            backToAmp();
        }

        if(curState == 4){
            score();
        }
    }

    private void preloadscore(){
        //if the arm has not been enabled, enable it
        if(!m_setArm){
        m_arm.setGoal(kBACK_AMP_PRESET.kArmPos);
        m_setArm = true;
        }
        //if the arm is finished and the wrist hasn't started, enable it
        if(m_setArm = true && m_arm.isArmSafe() && !m_setWrist){
        m_wrist.setGoal(kBACK_AMP_PRESET.kWristPos);
        m_setWrist = true;
        }

        //if the arm and the wrist are done, shoot the note
        if(m_setArm && m_arm.isArmSafe() && m_setWrist && m_wrist.isWristSafe()){
            m_intake.setVel(1.0);
            m_flywheel.setVel(1.0);
            m_timer.start();
        }
        if(m_timer.hasElapsed(0.3)){
            m_intake.setVel(0);
            m_flywheel.setVel(0);
            m_setArm = false;
            m_setWrist = false;
            curState = 1;
            
        }

    }

    private void intakePosition(){
        if(!m_setArm){
        m_arm.setGoal(kINTAKE_PRESET.kArmPos);
        m_setArm = true;
        }
        //if the arm is finished and the wrist hasn't started, enable it
        if(m_setArm = true && m_arm.isArmSafe() && !m_setWrist){
        m_wrist.setGoal(kINTAKE_PRESET.kWristPos);
        m_setWrist = true;
        }

        if(m_setArm && m_arm.isArmSafe() && m_setWrist && m_wrist.isWristSafe()){
            m_intake.setVel(1.0);
            m_setArm = false;
            m_setWrist = false;
            curState = 2;       
        }
    }

    private void groundPickup(){

        m_driveTrain.setLeftSpeed(0.1);
        m_driveTrain.setRightSpeed(0.1);

        if(m_intake.hasNote()){
        m_intake.setVel(0);
        m_driveTrain.setLeftSpeed(0);
        m_driveTrain.setRightSpeed(0);
        m_timer.reset();
        curState = 3;
        }
        
    }

    private void backToAmp(){
        m_driveTrain.setLeftSpeed(-0.1);
        m_driveTrain.setRightSpeed(-0.1);
        if(m_timer.hasElapsed(1)){
            curState = 4;
        }
    }

    private void score(){
            //if the arm has not been enabled, enable it
        if(!m_setArm){
        m_arm.setGoal(kBACK_AMP_PRESET.kArmPos);
        m_setArm = true;
        }
        //if the arm is finished and the wrist hasn't started, enable it
        if(m_setArm = true && m_arm.isArmSafe() && !m_setWrist){
        m_wrist.setGoal(kBACK_AMP_PRESET.kWristPos);
        m_setWrist = true;
        }

        //if the arm and the wrist are done, shoot the note
        if(m_setArm && m_arm.isArmSafe() && m_setWrist && m_wrist.isWristSafe()){
            m_intake.setVel(1.0);
            m_flywheel.setVel(1.0);
            m_timer.start();
        }
        if(m_timer.hasElapsed(0.3)){
            m_intake.setVel(0);
            m_flywheel.setVel(0);
            m_setArm = false;
            m_setWrist = false;
            curState = 5;
            
        }

    }


    //is finished only determines if the command is done or not!
    //write code in execute
    @Override
    public boolean isFinished() {
        if (curState == 5) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("double amp shot done");
        m_intake.setVel(0.0);
        m_flywheel.setSpeed(0.0);
        m_timer.stop();
        if(!m_intake.hasNote()) {
        m_wrist.setGoal(kSTOW_AWAY_PRESET.kWristPos);
        m_arm.setGoal(kSTOW_AWAY_PRESET.kArmPos);
        }
    }
    
}

