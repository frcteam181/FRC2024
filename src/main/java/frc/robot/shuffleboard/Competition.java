package frc.robot.shuffleboard;

import static frc.robot.Constants.kARM;
import static frc.robot.Constants.kCLIMBER;
import static frc.robot.Constants.kDRIVE_TRAIN;
import static frc.robot.Constants.kINTAKE;
import static frc.robot.Constants.kWRIST;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.Auto.PreLoadOnly;
import frc.robot.commands.Auto.PreLoad_n_Taxi;
import frc.robot.commands.Auto.TWOPIECE;
import frc.robot.commands.Auto.TaxiOnly;
import frc.robot.commands.Auto.TimeTaxiOnly;
import frc.robot.commands.Auto.a_PreLoadOnly;
import frc.robot.commands.Auto.a_PreLoad_n_Taxi;
import frc.robot.commands.Auto.a_TaxiOnly;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Competition {

    private ShuffleboardTab m_tab;

    private SendableChooser<Command> m_autoChooser;

    private Arm m_arm;
    private Climber m_climber;
    private Wrist m_wrist;
    private Intake m_intake;
    private DriveTrain m_driveTrain;

    public Competition() {

        m_arm = kARM;
        m_wrist = kWRIST;
        m_intake = kINTAKE;
        m_driveTrain = kDRIVE_TRAIN;
        m_climber = kCLIMBER;

        m_tab = Shuffleboard.getTab("Competition");

        m_autoChooser = new SendableChooser<Command>();

        m_autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        m_autoChooser.addOption("Do Nothing", new DoNothing());        
        m_autoChooser.addOption("Pre-Load Only", new a_PreLoadOnly());
        m_autoChooser.addOption("Taxi Only", new a_TaxiOnly());
        m_autoChooser.addOption("Pre-Load & Taxi", new a_PreLoad_n_Taxi());
        m_tab.add("Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(2, 1);

        m_tab.addBoolean("Has Note", m_intake::hasNote).withPosition(0, 0).withSize(1, 1);
        m_tab.addNumber("Arm Pos", m_arm::getPosDeg).withPosition(1,0);
        m_tab.addNumber("Wrist Pos", m_wrist::getPosDeg).withPosition(2, 0);
        m_tab.addNumber("DT Left Pos (in)", m_driveTrain::getLeftPosIn).withPosition(1, 1);
        m_tab.addNumber("DT Right Pos (in)", m_driveTrain::getRightPosIn).withPosition(2, 1);

        m_tab.addBoolean("Left Climber", m_climber::isLeftHome).withPosition(0, 1).withSize(1, 1);
        m_tab.addBoolean("Right Climber", m_climber::isRightHome).withPosition(0, 2).withSize(1, 1);

    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }
    
}
