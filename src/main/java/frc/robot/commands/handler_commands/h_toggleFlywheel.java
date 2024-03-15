package frc.robot.commands.handler_commands;

import static frc.robot.Constants.kHANDLER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class h_toggleFlywheel extends Command {

    private Handler m_handler;

    public h_toggleFlywheel() {

        m_handler = kHANDLER;

        //addRequirements(m_handler);

    }

    @Override
    public void initialize() {
        System.out.println("Flywheel Toggle Command Ran");
        m_handler.toggleFlywheel();
    }
    
}
