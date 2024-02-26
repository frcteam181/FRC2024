package frc.robot.commands.intake_actions;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class teleIntake extends Command {

    private Intake m_intake;
    private FlyWheel m_flywheel;

    public teleIntake() {

        m_intake = kINTAKE;
        m_flywheel = kFLYWHEEL;

        addRequirements(m_intake, m_flywheel);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}
