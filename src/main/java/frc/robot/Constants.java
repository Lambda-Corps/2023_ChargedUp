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
    
    public static final int kCountsPerRev = 2048;    // Encoder counts per revolution of the motor shaft.
	public static final double kSensorGearRatio = 4.17; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
														// drivetrain, encoders mount 1:1 with the gearbox shaft.
	public static final double kGearRatio = 4.17;   // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
														// of on the gearbox.
	public static final double kWheelRadiusInches = 2.9;
	public static final int k100msPerSecond = 10;
	public final static int kEncoderUnitsPerRotation = 36465;
	public final static double kEncoderTicksPerDegree = kEncoderUnitsPerRotation / 360;
    public static final double kEncoderTicksPerInch = (kCountsPerRev * kGearRatio) / (2 * Math.PI * kWheelRadiusInches);
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
    

    /////////////////// Human Interface ///////////////////////////
    public static final int DRIVER_LEFT_AXIS = 1;
    public static final int DRIVER_RIGHT_AXIS = 4;
    /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;

    /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
    /* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_DriveMM = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
}
