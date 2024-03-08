package frc.robot.commands.climber_actions;

import static frc.robot.Constants.kCLIMBER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class moveClimber extends Command {

    private Climber m_climber;

    private double m_speed, m_direction;

    public moveClimber(double speed, boolean up) {

        m_climber = kCLIMBER;
        
        if(up) {
            m_direction = 1.0;
        } else {
            m_direction = -1.0;
        }

        addRequirements(m_climber);

    }

    @Override
    public void execute() {
        m_climber.moveClimber(m_direction * m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.enableBreaks();
    }
    
}
