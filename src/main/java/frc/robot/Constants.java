package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.MyUtils.Gains;
import frc.robot.MyUtils.PresetValues;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Constants {

    //// SparkMaxs (CAN ID) ////
    // RIO = 0
    // PDH = 1
    public static final int kLEFT_LEADER_ID = 2;
    public static final int kLEFT_FOLLOWER_ID = 3;
    public static final int kRIGHT_LEADER_ID = 4;
    public static final int kRIGHT_FOLLOWER_ID = 5;

    public static final int kLEFT_ARM_ID = 6;
    public static final int kRIGHT_ARM_ID = 7;

    public static final int kLEFT_WRIST_ID = 8;
    public static final int kRIGHT_WRIST_ID = 9;

    public static final int kLEFT_FLYWHEEL_ID = 10;
    public static final int kRIGHT_FLYWHEEL_ID = 11;

    public static final int kINTAKE_ID = 12;

    public static final int kGYRO_ID = 13;

    /// RoboRIO DIO Channels ////
    public static final int kNOTE_BEAM_CHANNEL = 0;

    //// Controllers and Joysticks ////
    public static final int kJOYSTICK_ONE = 0;
    public static final int kJOYSTICK_TWO = 1;
    public static final int kCONTROLLER = 2;
    public static final int kOPERATOR = 3;

    //// Subsystem Settings ////

    // Drive Train (meter)

    public static final Gains kDT_DRIVE_POS_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0);
    public static final Gains kDT_TURN_POS_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1);
    public static final Gains kDT_DRIVE_VEL_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2);
    public static final Gains kDT_TURN_VEL_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 3);

    public static final int kLEFT_DRIVETRAIN_CURRENT_LIMIT = 50;
    public static final int kRIGHT_DRIVETRAIN_CURRENT_LIMIT = 50;

    public static final double kDRIVETRAIN_GEAR_RATIO = (0.13585);
    public static final double kWHEEL_DIAMETER_METER = (Units.inchesToMeters(4.0));
    public static final double kDRIVETRAIN_POS_FACTOR_METER = (kDRIVETRAIN_GEAR_RATIO * kWHEEL_DIAMETER_METER * Math.PI); // m
    public static final double kDRIVETRAIN_VEL_FACTOR_METER = (kDRIVETRAIN_POS_FACTOR_METER/60.0); // m/sec

    public static final double kDRIVE_THRESHOLD = 0.04;
    public static final double kDRIVE_DEADZONE = 0.045;
    public static final double kDRIVE_TURN_THRESHOLD = 0.045;
    public static final double kTURN_DEADZONE = 0.05;
    public static final int kDRIVE_CURVE_ODD_NUMBER = 3; // only use odd numbers
    

    // Arm (rad)
    public static final Gains kARM_GAINS = new Gains(2.0, 0.0, 0.1, 0.0, 0, -1.0, 1.0, 0.0, Math.toRadians(120.0), Math.toRadians(50.0), 0.0, 0);
    //public static final Gains kARM_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, Math.toRadians(120.0), Math.toRadians(50.0), 0.0, 0);

    public static final int kLEFT_ARM_CURRENT_LIMIT = 70;
    public static final int kRIGHT_ARM_CURRENT_LIMIT = 70;

    public static final double kARM_GEAR_RATIO = (1.0/1.0);
    public static final double kARM_POS_FACTOR_RAD = Math.toRadians(kARM_GEAR_RATIO * 360.0); // rad
    public static final double kARM_VEL_FACTOR_RAD = (kARM_POS_FACTOR_RAD/60.0); // rad/sec

    public static final double kARM_ZERO_OFFSET = 5.83; // rad
    public static final double kZERO_ARM = 0;//Math.toRadians(28.72); // On bumper reading = 0

    public static final double kMAX_ARM_POS_RAD = Math.toRadians(100.0); // rad
    public static final double kMIN_ARM_POS_RAD = Math.toRadians(0.0); // rad

    public static final double kARM_KS = 0.0;
    public static final double kARM_KG = 0.1;
    public static final double kARM_KV = 0.5;

    // Wrist (rad)
    public static final Gains kWRIST_GAINS = new Gains(0.55, 0.0, 0.0, 0.0, 0, -1.0, 1.0, 0.0, Math.toRadians(900), Math.toRadians(500), 0.0, 0); //rad


    public static final int kLEFT_WRIST_CURRENT_LIMIT = 40;
    public static final int kRIGHT_WRIST_CURRENT_LIMIT = 40;

    public static final double kWRIST_GEAR_RATIO = (1.0/1.0);
    public static final double kWRIST_POS_FACTOR_RAD = (Math.toRadians(kWRIST_GEAR_RATIO * 360.0)); // rad
    public static final double kWRIST_VEL_FACTOR_RAD = (kWRIST_POS_FACTOR_RAD/60.0); // rad/sec

    public static final double kWRIST_ZERO_OFFSET = Math.toRadians(25.0); // rad
    public static final double kZERO_WRIST = Math.toRadians(246.5);

    public static final double kMAX_WRIST_POS_RAD = Math.toRadians(50.0); // rad
    public static final double kMIN_WRIST_POS_RAD = Math.toRadians(-120.0); // rad

    // Flywheel
    public static final Gains kFLYWHEEL_GAINS = new Gains(0.0001, 0.0, 0.0, 0.000185, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0);

    public static final int kLEFT_FLYWHEEL_CURRENT_LIMIT = 30;
    public static final int kRIGHT_FLYWHEEL_CURRENT_LIMIT = 30;

    public static final double kFLYWHEEL_POS_FACTOR_RAD = 1; // RPM
    public static final double kFLYWHEEL_VEL_FACTOR_RAD = 1; // RPM

    public static final double kFLYWHEEL_SHOOT_POWER = 0;

    // Intake
    public static final Gains kINTAKE_GAINS = new Gains(0.00002, 0.0, 0.0/*0.0025*/, 0.0001, 0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0);

    public static final int kINTAKE_CURRENT_LIMIT = 30;

    public static final double kINTAKE_POS_FACTOR_RAD = 1; // RPM
    public static final double kINTAKE_VEL_FACTOR_RAD = 1; // RPM

    public static final double kINTAKE_POWER = 1.0; //90000.0
    public static final double kINTAKE_FEED_POWER = 1.0; //3500.0

    //Presets

    public static final PresetValues kINTAKE_PRESET = new PresetValues(-5.0, 0.0, true);
    public static final PresetValues kSTOW_AWAY_PRESET = new PresetValues(-2.4, 16.5, true);
    
    public static final PresetValues kBACK_AMP_PRESET = new PresetValues(5.6, 94.5, true);
    public static final PresetValues kFORWARD_AMP_PRESET = new PresetValues(-0.7, 49.0, true);
    
    public static final PresetValues kBACK_HIGH_SPEAKER_PRESET = new PresetValues(-70.0, 81.0, true);
    public static final PresetValues kBACK_LOW_SPEAKER_PRESET = new PresetValues(-2.4, 16.5, true);
    public static final PresetValues kFORWARD_HIGH_SPEAKER_PRESET = new PresetValues(-110, 49.0, true);

    /// MUST BE LAST TO LOAD ///
    //// Subsystems ////
    public static final DriveTrain kDRIVE_TRAIN = new DriveTrain(false);
    public static final Arm kARM = new Arm(true);
    public static final Wrist kWRIST = new Wrist(false); 
    public static final FlyWheel kFLYWHEEL = new FlyWheel(false);
    public static final Intake kINTAKE = new Intake(false);
}
