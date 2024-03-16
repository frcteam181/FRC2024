package frc.robot.commands.handler_commands;

import static frc.robot.Constants.kHANDLER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class h_holdIntake extends Command {

    private Handler m_handler;

    public h_holdIntake() {

        m_handler = kHANDLER;

        addRequirements(m_handler);

    }

    @Override
    public void initialize() {
        m_handler.setIntakeState(1);
    }

    /*@Override
    public boolean isFinished() {
        if(m_handler.getIntakeState() != 1) {
            return true;
        } else {
            return false;
        }
    }*/

    /*@Override
    public void end(boolean interrupted) {
        if(m_handler.getIntakeState() == 1) {
            m_handler.setIntakeState(0);
        }
    }*/

    @Override
    public void end(boolean interrupted) {
        m_handler.setIntakeState(0);
    }


}
