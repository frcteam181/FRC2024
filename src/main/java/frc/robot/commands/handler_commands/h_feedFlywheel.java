package frc.robot.commands.handler_commands;

import static frc.robot.Constants.kHANDLER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class h_feedFlywheel extends Command {

    private Handler m_handler;

    public h_feedFlywheel() {

        m_handler = kHANDLER;

        addRequirements(m_handler);

    }

    @Override
    public void initialize() {
        m_handler.setIntakeState(4);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
