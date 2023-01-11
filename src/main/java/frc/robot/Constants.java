// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    ////////////////////// CAN IDs //////////////////////
    public final static int LEFT_TALON_LEADER    = 1;
    public final static int LEFT_TALON_FOLLOWER  = 3;
    public final static int RIGHT_TALON_LEADER   = 2;
    public final static int RIGHT_TALON_FOLLOWER = 4; 
    public final static int CAN_ID_FIVE = 5; 
    public final static int CAN_ID_SIX = 6;
    public final static int CAN_ID_SEVEN = 7;
    public final static int CAN_ID_EIGHT = 8;
    public final static int CAN_ID_NINE = 9;
    public final static int CAN_ID_TEN = 10;
    public final static int CAN_ID_ELEVEN = 11;
    public final static int CAN_ID_TWELVE = 12;
    public final static int CAN_ID_THIRTEEN = 13;

    //////////////////// Roborio DIO ////////////////////

    ////////// Robot Physical Characteristics ///////////
    public static final double k_robot_mass = 55.3; // MUST BE CHANGED, THIS IS 2022 number
    public static final double k_dt_gear_ratio = 1.0; // MUST BE CHANGED TO OUR ACTUAL GEARBOX
    public static final double k_dt_wheel_radius_inches = 3.0;
    public static final double k_dt_track_width_meters = .546; // MUST BE CHANGED, THIS IS 2022

    ////////////////////// DriveTrain Specific constants ///////////////////
    public static final double k_dt_neutral_deadband = .1;  // MUST BE TUNED
    public static final double k_dt_peak_output_forward = 1.0; // MUST BE TUNED
    public static final double k_dt_peak_output_reverse = -1.0;
}
