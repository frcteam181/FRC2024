package frc.robot.commands.Auto;

import static frc.robot.Constants.kDRIVE_TRAIN;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class a_TaxiOnly extends SequentialCommandGroup {

    private DriveTrain m_driveTrain;
    
    public a_TaxiOnly() {

        m_driveTrain = kDRIVE_TRAIN;

        addRequirements(m_driveTrain);

        addCommands(
            m_driveTrain.setSpeedCommand(0.2),
            new WaitCommand(3),
            m_driveTrain.setSpeedCommand(0.0)
        );

    }

}
