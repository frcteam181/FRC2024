package frc.robot;

import edu.wpi.first.math.util.Units;
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

    // Drive Train
    public static final int kDRIVE_PID_SLOT_ID = 0;
    public static final Gains kDRIVE_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0);

    public static final int kLEFT_DRIVETRAIN_CURRENT_LIMIT = 70;
    public static final int kRIGHT_DRIVETRAIN_CURRENT_LIMIT = 70;

    public static final double kDRIVETRAIN_GEAR_RATIO = (1.0/1.0);
    public static final double kWHEEL_DIAMETER_METER = (Units.inchesToMeters(4.0));
    public static final double kDRIVETRAIN_POS_FACTOR_METER = (kDRIVETRAIN_GEAR_RATIO * kWHEEL_DIAMETER_METER * Math.PI); // m
    public static final double kDRIVETRAIN_VEL_FACTOR_METER = (kDRIVETRAIN_POS_FACTOR_METER/60.0); // m/sec

    public static final double kDRIVE_THRESHOLD = 0.01;
    public static final int kDRIVE_CURVE_ODD_NUMBER = 3; // only use odd numbers
    public static final double kDRIVE_DEADZONE = 0.01;

    // Arm
    public static final int kARM_PID_SLOT_ID = 0;
    public static final Gains kARM_GAINS = new Gains(0.8, 0.0001, 0.0, 0.0, 0, -1.0, 1.0);

    public static final int kLEFT_ARM_CURRENT_LIMIT = 70;
    public static final int kRIGHT_ARM_CURRENT_LIMIT = 70;

    public static final double kARM_GEAR_RATIO = (1.0/300.0); /////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public static final double kARM_POS_FACTOR_RAD = Math.toRadians(kARM_GEAR_RATIO * 360.0); // rad
    public static final double kARM_VEL_FACTOR_RAD = (kARM_POS_FACTOR_RAD/60.0); // rad/sec

    public static final double kMAX_ARM_VEL_RAD = Math.toRadians(120); // rad/s
    public static final double kMAX_ARM_ACC_RAD = Math.toRadians(35); // rad/s^2

    public static final double kMAX_ARM_POS_RAD = 0.0; // rad
    public static final double kMIN_ARM_POS_RAD = 0.0; // rad

    public static final double kARM_KS = 0.0;
    public static final double kARM_KG = 0.0;
    public static final double kARM_KV = 0.0;

    // Wrist
    public static final int kWRIST_PID_SLOT_ID = 0;
    public static final Gains kWRIST_GAINS = new Gains(0.55, 0.0, 0.0, 0.0, 0, -1.0, 1.0);

    public static final int kLEFT_WRIST_CURRENT_LIMIT = 40;
    public static final int kRIGHT_WRIST_CURRENT_LIMIT = 40;

    public static final double kWRIST_GEAR_RATIO = (1.0/25.0); ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public static final double kWRIST_POS_FACTOR_RAD = (Math.toRadians(kWRIST_GEAR_RATIO * 360.0)); // rad
    public static final double kWRIST_VEL_FACTOR_RAD = (kWRIST_POS_FACTOR_RAD/60.0); // rad/sec

    public static final double kMAX_WRIST_VEL_RAD = Math.toRadians(900); // rad/s
    public static final double kMAX_WRIST_ACC_RAD = Math.toRadians(500); // rad/s^2

    public static final double kMAX_WRIST_POS_RAD = 0.0; // rad
    public static final double kMIN_WRIST_POS_RAD = 0.0; // rad

    // Fly Wheel
    public static final int kFLYWHEEL_PID_SLOT_ID = 0;
    public static final Gains kFLYWHEEL_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0);

    public static final int kLEFT_FLYWHEEL_CURRENT_LIMIT = 70;
    public static final int kRIGHT_FLYWHEEL_CURRENT_LIMIT = 70;

    public static final double kFLYWHEEL_GEAR_RATIO = (1.0/3.0);
    public static final double kFLYWHEEL_POS_FACTOR_RAD = (Math.toRadians(kFLYWHEEL_GEAR_RATIO * 360.0)); // rad
    public static final double kFLYWHEEL_VEL_FACTOR_RAD = (kFLYWHEEL_POS_FACTOR_RAD/60.0); // rad/sec

    public static final double kMAX_FLYWHEEL_VEL_RAD = 0.0; // rad/s
    public static final double kMAX_FLYWHEEL_ACC_RAD = 0.0; // rad/s^2

    // Intake
    public static final int kINTAKE_PID_SLOT_ID = 0;
    public static final Gains kINTAKE_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, -1.0, 1.0);

    public static final int kINTAKE_CURRENT_LIMIT = 20;

    public static final double kINTAKE_GEAR_RATIO = (1.0/3.0);
    public static final double kINTAKE_POS_FACTOR_RAD = Math.toRadians(kINTAKE_GEAR_RATIO * 360.0); // rad
    public static final double kINTAKE_VEL_FACTOR_RAD = kINTAKE_POS_FACTOR_RAD/60.0; // rad/sec

    public static final double kMAX_INTAKE_VEL_RAD = 0.0; // rad/s
    public static final double kMAX_INTAKE_ACC_RAD = 0.0; // rad/s^2

    /// MUST BE LAST TO LOAD ///
    //// Subsystems ////
    public static final DriveTrain kDRIVE_TRAIN = new DriveTrain(false);
    public static final Arm kARM = new Arm(false);
    public static final Wrist kWRIST = new Wrist(true);
    public static final FlyWheel kFLYWHEEL = new FlyWheel(false);
    public static final Intake kINTAKE = new Intake(false);
}
