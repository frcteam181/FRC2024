package frc.robot.commands.flywheel_actions;

import static frc.robot.Constants.kFLYWHEEL;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class toggleFlywheel extends Command {

    private Flywheel m_flywheel;

    public toggleFlywheel() {
        m_flywheel = kFLYWHEEL;
    }

    @Override
    public void initialize() {
        m_flywheel.toggleFlywheel();
    }
    
}
