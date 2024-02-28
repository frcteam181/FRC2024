package frc.robot.commands.intake_actions;

import static frc.robot.Constants.kINTAKE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class feedFlywheel extends Command {

    private Intake m_intake;

    public feedFlywheel() {

        m_intake = kINTAKE;

        addRequirements(m_intake);

    }

    @Override
    public void initialize() {
        m_intake.toggleFeed();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.toggleFeed();
    }
    
}
