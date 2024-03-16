package frc.robot.commands.handler_commands;

import static frc.robot.Constants.kHANDLER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class h_toggleIntake extends Command {

    private Handler m_handler;

    public h_toggleIntake() {

        m_handler = kHANDLER;

        addRequirements(m_handler);

    }

    @Override
    public void initialize() {
        m_handler.toggleIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
