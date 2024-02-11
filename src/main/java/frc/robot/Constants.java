package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Constants {

    //// Subsystems ////
    public static final DriveTrain kDRIVE_TRAIN = new DriveTrain(false);
    public static final Arm kARM = new Arm(false);
    public static final Wrist kWRIST = new Wrist(false);
    public static final FlyWheel kFLYWHEEL = new FlyWheel(false);
    public static final Intake kINTAKE = new Intake(false);

    //// SparkMaxs (CAN ID) ////
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

    //// Controllers and Joysticks ////
    public static final int kJOYSTICK_ONE = 0;
    public static final int kJOYSTICK_TWO = 1;
    public static final int kCONTROLLER = 2;
    public static final int kOPERATOR = 3;

    //// Subsystem Settings ////

    // Drive Train
    public static final double kDRIVE_THRESHOLD = 0;
    public static final int kDRIVE_CURVE_ODD_NUMBER = 3;
    public static final double kDRIVE_DEADZONE = 0.01;

    // Arm
    public static final int kARM_PID_SLOT_ID = 0;
    public static final Gains kARM_GAINS = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final int kLEFT_ARM_CURRENT_LIMIT = 70;
    public static final int kRIGHT_ARM_CURRENT_LIMIT = 70;

    public static final double kARM_GEAR_RATIO = (1.0/300.0);
    public static final double kARM_POS_FACTOR_RAD = Math.toRadians(kARM_GEAR_RATIO * 360.0); // rad
    public static final double kARM_VEL_FACTOR_RAD = kARM_POS_FACTOR_RAD/60.0; // rad/sec

    public static final double kMAX_ARM_VEL_RAD = 0; // rad/s
    public static final double kMAX_ARM_ACC_RAD = 0; // rad/s^2

    public static final double kMAX_ARM_POS_RAD = 0; // rad
    public static final double kMIN_ARM_POS_RAD = 0; // rad

    // Wrist
    public static final int kINTAKE_PID_SLOT_ID = 0;
    public static final Gains kINTAKE_GAINS = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final int kLEFT_WRIST_CURRENT_LIMIT = 70;
    public static final int kRIGHT_WRIST_CURRENT_LIMIT = 70;

    public static final double kWRIST_GEAR_RATIO = (1.0/300.0);
    public static final double kWRIST_POS_FACTOR_RAD = Math.toRadians(kARM_GEAR_RATIO * 360.0); // rad
    public static final double kWRIST_VEL_FACTOR_RAD = kARM_POS_FACTOR_RAD/60.0; // rad/sec

    public static final double kMAX_WRIST_VEL_RAD = 0; // rad/s
    public static final double kMAX_WRIST_ACC_RAD = 0; // rad/s^2

    public static final double kMAX_WRIST_POS_RAD = 0; // rad
    public static final double kMIN_WRIST_POS_RAD = 0; // rad

    // Fly Wheel

    // Intake
    
}
