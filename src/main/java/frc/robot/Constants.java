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
    public final static int ARM_MOTOR = 5; 
    public final static int WRIST_MOTOR = 6;
    public final static int CAN_ID_SEVEN = 7;
    public final static int CAN_ID_EIGHT = 8;
    public final static int CAN_ID_NINE = 9;
    public final static int CAN_ID_TEN = 10;
    public final static int CAN_ID_ELEVEN = 11;
    public final static int CAN_ID_TWELVE = 12;
    public final static int CAN_ID_THIRTEEN = 13;

    /////////////////// Roborio DIO ///////////////////
    public final static int ARM_REVERSE_LIMIT_SWITCH = 1;
    public final static int ARM_FORWARD_LIMIT_SWITCH = 0;
    public final static int WRIST_REVERSE_LIMIT_SWITCH = 3;
    public final static int WRIST_FORWARD_LIMIT_SWITCH = 2;
    

    /////////////////// Human Interface ///////////////////
    public static final int DRIVER_LEFT_AXIS = 1;
    public static final int DRIVER_RIGHT_AXIS = 4;
    public static final int PARTNER_LEFT_AXIS = 0;
    public static final int PARTNER_RIGHT_AXIS = 4;

}
