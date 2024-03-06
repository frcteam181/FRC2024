package frc.robot.commands.Auto;

import static frc.robot.Constants.kDRIVE_TRAIN;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class TimeTaxiOnly extends SequentialCommandGroup {

    private DriveTrain m_driveTrain;

    public TimeTaxiOnly(double speed, double sec) {

        m_driveTrain = kDRIVE_TRAIN;

        addCommands(
            m_driveTrain.setSpeedCommand(speed),
            new WaitCommand(sec),
            m_driveTrain.setSpeedCommand(0)
        );

    }
}
