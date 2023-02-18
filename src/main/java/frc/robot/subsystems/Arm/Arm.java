// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX m_arm_motor, m_wrist_motor;
  DigitalInput  m_arm_forward_limit, m_arm_reverse_limit, m_wrist_reverse_limit, m_wrist_forward_limit;
  DoublePublisher m_arm_position, m_wrist_position;

  DoubleSolenoid m_gripper;

  public ArmState arm_state;
  public ArmControlMode arm_control_mode;
  public ArmPosition arm_position;
  public ArmTask arm_task;

  public enum ArmState {
    Active,
    Inactive,
  }

  public enum ArmControlMode {  // Mostly for debugging information, could be used as an arm safety measure in emergencies
    Manual,
    Automatic,
  }

  public enum ArmPosition {
    Stowed,
    GroundPickup,
    SubstationPickup,
    ScoreLow,
    ScoreConeMid,
    ScoreConeHigh,
    ScoreCubeMid,
    ScoreCubeHigh,
  }

  public enum ArmTask {
    HoldPosition,
    MoveToPosition,
    MoveManually,
    Stop,
  }

  ///////// Constants ///////////////
  // Constant values for ARM movement, must be researched and tuned via tuner
  final double ARM_FORWARD_SPEED = .4;
  final double ARM_REVERSE_SPEED = -.4;
  final double WRIST_FORWARD_SPEED = .5;
  final double WRIST_REVERSE_SPEED = -.3;
  final double ARM_GEAR_RATIO = 10 * 4 * 4;
  final double WRIST_GEAR_RATIO = 7 * 5 * 5;
  final int WRIST_REVERSE_SOFT_LIMIT = 1500;// TODO TUNE THESE
  final int WRIST_FORWARD_SOFT_LIMIT = 123000; // 264,904, 265763, 266612
  final int ARM_REVERSE_SOFT_LIMIT = 1000; 
  final int ARM_FORWARD_SOFT_LIMIT = 46000;
  // final int ARM_FORWARD_SOFT_LIMIT = (int)(2048 * ARM_GEAR_RATIO * 1/6); // 60 degrees rotation
  final double WRIST_MAX_STATOR_CURRENT = 20;
  final double ARM_MAX_STATOR_CURRENT = 20;
  final int ARM_MM_FORWARD_SLOT = 0;
  final int ARM_MM_REVERSE_SLOT = 1;
  final int ARM_MANUAL_FORWARD_SLOT = 2;
  final int ARM_MANUAL_REVERSE_SLOT = 3;
  final int WRIST_MM_FORWARD_SLOT = 0;
  final int WRIST_MM_REVERSE_SLOT = 1;
  final int WRIST_MANUAL_FORWARD_SLOT = 2;
  final int WRIST_MANUAL_REVERSE_SLOT = 3;
  final DoubleSolenoid.Value GRIPPER_CONTRACT = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value GRIPPER_EXPAND   = DoubleSolenoid.Value.kReverse;

  // The wrist travels 133 degrees total, for manual steps try to go 1 degree at a time
  final int WRIST_POSITION_STEP = (int)(WRIST_FORWARD_SOFT_LIMIT / 65);
  // The arm travels 52 degrees total, for manual steps try one degree at a time
  final int ARM_POSITION_STEP = (int)(ARM_FORWARD_SOFT_LIMIT / 26); 

  /*
   * KP Calculations are: 
   * Desired Percent * full throttle / error
   * That means, we want to saturate the feedback (full recovery) with an error value
   */
  final double ARM_MANUAL_FORWARD_KP = 3.2; // Tuned manually (ARM_FORWARD_SPEED * 1023) / 2048;
  final double ARM_MANUAL_FORWARD_KI = 0;
  final double ARM_MANUAL_FORWARD_KD = 0;
  final double ARM_MANUAL_FORWARD_KF = 0;
  final double ARM_MANUAL_FORWARD_FF = .3;
  final double ARM_MANUAL_REVERSE_KP = (ARM_REVERSE_SPEED * 1023) / 2048;
  final double ARM_MANUAL_REVERSE_KI = 0;
  final double ARM_MANUAL_REVERSE_KD = 0;
  final double ARM_MANUAL_REVERSE_KF = 0;
  final double ARM_MANUAL_REVERSE_FF = -.3;
  final double WRIST_MANUAL_FORWARD_KP = (WRIST_FORWARD_SPEED * 1023) / 2048;
  final double WRIST_MANUAL_FORWARD_KI = 0;
  final double WRIST_MANUAL_FORWARD_KD = 0;
  final double WRIST_MANUAL_FORWARD_KF = 0;
  final double WRIST_MANUAL_FORWARD_FF = .075;
  final double WRIST_MANUAL_REVERSE_KP = (WRIST_REVERSE_SPEED * 1023) / 2048;
  final double WRIST_MANUAL_REVERSE_KI = 0;
  final double WRIST_MANUAL_REVERSE_KD = 0;
  final double WRIST_MANUAL_REVERSE_KF = 0;

  // Encoder Measurements for the relevant scoring positions
  final int ARM_STOW = 0;
  final int WRIST_STOW = 0;
  final int ARM_GROUND_PICKUP = 0; // TODO - 18 Feb, measure all of these and save
  final int WRIST_GROUND_PICKUP = 0;
  final int ARM_SUBSTATION = 0;
  final int WRIST_SUBSTATION = 0;
  final int ARM_SCORE_LOW = 0;
  final int WRIST_SCORE_LOW = 0;
  final int ARM_CONE_MID = 0;
  final int WRIST_CONE_MID = 0;
  final int ARM_CONE_HIGH = 0;
  final int WRIST_CONE_HIGH = 0;
  final int ARM_CUBE_HIGH = 0;
  final int WRIST_CUBE_HIGH = 0;
  final int ARM_CUBE_MID = 0;
  final int WRIST_CUBE_MID = 0;

  /** Creates a new Arm. */
  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive input rotates to FRONT of robot
    m_arm_motor = new WPI_TalonFX(ARM_MOTOR);
    m_wrist_motor = new WPI_TalonFX(WRIST_MOTOR);

    // Factory default the talons
    m_arm_motor.configFactoryDefault();
    m_wrist_motor.configFactoryDefault();
    
    TalonFXConfiguration arm_config = new TalonFXConfiguration();
    TalonFXConfiguration wrist_config = new TalonFXConfiguration();

    // Setup the PID control variables for arm movement
    SlotConfiguration arm_manual_forward = arm_config.slot2;
    arm_manual_forward.allowableClosedloopError = ARM_POSITION_STEP / 2;
    // arm_manual_forward.closedLoopPeakOutput = ARM_FORWARD_SPEED;
    arm_manual_forward.closedLoopPeakOutput = .5;
    arm_manual_forward.closedLoopPeriod = 1;
    arm_manual_forward.kP = ARM_MANUAL_FORWARD_KP;
    arm_manual_forward.kI = ARM_MANUAL_FORWARD_KI;
    arm_manual_forward.kD = ARM_MANUAL_FORWARD_KD;
    arm_manual_forward.kF = ARM_MANUAL_FORWARD_KF;
    arm_config.slot2 = arm_manual_forward;

    SlotConfiguration arm_manual_reverse = arm_config.slot3;
    arm_manual_reverse.allowableClosedloopError = ARM_POSITION_STEP / 2;
    arm_manual_reverse.closedLoopPeakOutput = ARM_REVERSE_SPEED;
    arm_manual_reverse.closedLoopPeriod = 1;
    arm_manual_reverse.kP = ARM_MANUAL_REVERSE_KP;
    arm_manual_reverse.kI = ARM_MANUAL_REVERSE_KI;
    arm_manual_reverse.kD = ARM_MANUAL_REVERSE_KD;
    arm_manual_reverse.kF = ARM_MANUAL_REVERSE_KF;
    arm_config.slot3 = arm_manual_reverse;

    SlotConfiguration wrist_manual_forward = wrist_config.slot2;
    wrist_manual_forward.allowableClosedloopError = WRIST_POSITION_STEP / 2;
    wrist_manual_forward.closedLoopPeakOutput = WRIST_FORWARD_SPEED;
    wrist_manual_forward.closedLoopPeriod = 1;
    wrist_manual_forward.kP = WRIST_MANUAL_FORWARD_KP;
    wrist_manual_forward.kI = WRIST_MANUAL_FORWARD_KI;
    wrist_manual_forward.kD = WRIST_MANUAL_FORWARD_KD;
    wrist_manual_forward.kF = WRIST_MANUAL_FORWARD_KF;
    wrist_config.slot2 = wrist_manual_forward;

    SlotConfiguration wrist_manual_reverse = wrist_config.slot3;
    wrist_manual_reverse.allowableClosedloopError = WRIST_POSITION_STEP / 2;
    wrist_manual_reverse.closedLoopPeakOutput = WRIST_REVERSE_SPEED;
    wrist_manual_reverse.closedLoopPeriod = 1;
    wrist_manual_reverse.kP = WRIST_MANUAL_REVERSE_KP;
    wrist_manual_reverse.kI = WRIST_MANUAL_REVERSE_KI;
    wrist_manual_reverse.kD = WRIST_MANUAL_REVERSE_KD;
    wrist_manual_reverse.kF = WRIST_MANUAL_REVERSE_KF;
    wrist_config.slot3 = wrist_manual_reverse;

    // arm_config.slot0 = new SlotConfiguration();
    // arm_config.slot1 = new SlotConfiguration();
    // arm_config.slot2 = m_arm_manual_forward;
    // arm_config.slot3 = m_arm_manual_reverse;
    // wrist_config.slot0 = new SlotConfiguration();
    // wrist_config.slot1 = new SlotConfiguration();
    // wrist_config.slot2 = m_wrist_manual_forward;
    // wrist_config.slot3 = m_wrist_manual_reverse;
   
    
    // Setup the ARM stage motor
    m_arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ARM_MM_FORWARD_SLOT, 0);
    arm_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_arm_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Configure limit switches
    arm_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    arm_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    arm_config.reverseSoftLimitEnable = true;
    arm_config.forwardSoftLimitEnable = true;
    arm_config.reverseSoftLimitThreshold = ARM_REVERSE_SOFT_LIMIT;
    arm_config.forwardSoftLimitThreshold = ARM_FORWARD_SOFT_LIMIT;

    // Set current limits for the ARM
    StatorCurrentLimitConfiguration stator_limit = arm_config.statorCurrLimit;
    stator_limit.currentLimit = ARM_MAX_STATOR_CURRENT;
    stator_limit.enable = true;
    stator_limit.triggerThresholdCurrent = ARM_MAX_STATOR_CURRENT + 1;
    arm_config.statorCurrLimit = stator_limit;
    
    // Set max speeds for output
    arm_config.peakOutputForward = ARM_FORWARD_SPEED;
    arm_config.peakOutputReverse = ARM_REVERSE_SPEED;

    // Configure the arm
    m_arm_motor.configAllSettings(arm_config, 10);

    // set arm inversion
    m_arm_motor.setInverted(TalonFXInvertType.Clockwise);
   
    //set arm brake mode 
    m_arm_motor.setNeutralMode(NeutralMode.Brake);

    // Configure the Wrist motor
    m_wrist_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ARM_MM_FORWARD_SLOT, 0);
    wrist_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_wrist_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_wrist_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Configure limit switches
    wrist_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.reverseSoftLimitEnable = true;
    wrist_config.forwardSoftLimitEnable = true;
    wrist_config.reverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
    wrist_config.forwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;

    // Set current limits for the Wrist
    stator_limit = wrist_config.statorCurrLimit;
    stator_limit.currentLimit = WRIST_MAX_STATOR_CURRENT;
    stator_limit.enable = true;
    stator_limit.triggerThresholdCurrent = WRIST_MAX_STATOR_CURRENT + 1;
    wrist_config.statorCurrLimit = stator_limit;
    
    // Set max speeds for output
    wrist_config.peakOutputForward = WRIST_FORWARD_SPEED;
    wrist_config.peakOutputReverse = WRIST_REVERSE_SPEED;
    // Configure the wrist
    m_wrist_motor.configAllSettings(wrist_config, 10);
    m_wrist_motor.setInverted(TalonFXInvertType.Clockwise);
    m_wrist_motor.setNeutralMode(NeutralMode.Brake);

    // Top and bottom arm limit switches, these DIOs are for the LEDs to light up when the limits are hit
    m_arm_forward_limit = new DigitalInput(ARM_FORWARD_LIMIT_SWITCH);
    m_arm_reverse_limit = new DigitalInput(ARM_REVERSE_LIMIT_SWITCH);
    
    m_wrist_forward_limit = new DigitalInput(WRIST_FORWARD_LIMIT_SWITCH);
    m_wrist_reverse_limit = new DigitalInput(WRIST_REVERSE_LIMIT_SWITCH);

    // Setup the network tables publishers to push data to the dashboard
    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
    m_arm_position = shuffleboard.getDoubleTopic("ArmEncoder").publish();
    m_wrist_position = shuffleboard.getDoubleTopic("WristEncoder").publish();

    m_gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GRIPPER_SOLENOID_CHANNEL_A, GRIPPER_SOLENOID_CHANNEL_B);
    // Set the gripper to contracted for our preload
    m_gripper.set(GRIPPER_CONTRACT);    
  }

  @Override
  public void periodic() {
    m_arm_position.set(m_arm_motor.getSelectedSensorPosition());
    m_wrist_position.set(m_wrist_motor.getSelectedSensorPosition());
  }

  public void moveArmManually(double bottom_speed, double top_speed) {
    m_arm_motor.set(ControlMode.PercentOutput, bottom_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, top_speed);

    if (m_arm_forward_limit.get() && bottom_speed < 0) {
      m_arm_motor.set(ControlMode.PercentOutput, 0);
    }

    if (m_arm_reverse_limit.get() && top_speed < 0) {
      m_wrist_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  // DO NOT USE DURING MATCHES!!! ONLY RUN IN THE PITS!!!
  public void reset_arm_pos() {
    arm_control_mode = ArmControlMode.Automatic;
    // while BOTH top and bottoms are false (not triggered)
    while (!m_arm_forward_limit.get() && !m_arm_reverse_limit.get()) {
      // only drive base if limit is not hit
      if (!m_arm_forward_limit.get()) {
        m_arm_motor.set(ControlMode.PercentOutput, -ARM_FORWARD_SPEED);
      }else {
        m_arm_motor.set(ControlMode.PercentOutput, 0);
      }

      // only drive upper stage if limit is not hit
      if (!m_arm_reverse_limit.get()) {
        m_wrist_motor.set(ControlMode.PercentOutput, -WRIST_FORWARD_SPEED);
      }else {
        m_wrist_motor.set(ControlMode.PercentOutput, 0);
      }
    }
    
    // Zero encoders once both stages are reset
    m_arm_motor.setSelectedSensorPosition(0);
    m_wrist_motor.setSelectedSensorPosition(0);
  }

  public WPI_TalonFX getBottomStageMotor() {
    return m_arm_motor;
  }

  public WPI_TalonFX getTopStageMotor() {
    return m_wrist_motor;
  }

  public ArmState getArmState() {
    return arm_state;
  }

  public ArmControlMode getArmControlMode() {
    return arm_control_mode;
  }

  public ArmPosition getArmPosition() {
    return arm_position;
  }

  public ArmTask getArmTask() {
    return arm_task;
  }

  public DigitalInput getBottomLimitSwitch() {
    return m_arm_forward_limit;
  }

  public DigitalInput getTopLimitSwitch() {
    return m_arm_reverse_limit;
  }

  private int radiansToNativeUnits(double radians) {
    double ticks = (radians / (2 *  Math.PI)) * 2048;
    return (int)ticks;
  }

  // Converts to native units per 100 ms
  private int velocityToNativeUnits(double radPerSec) {
    double radPer100ms = radPerSec / 1000;
    return radiansToNativeUnits(radPer100ms);
  }

  public boolean getArmForwardLimit(){
    return m_arm_forward_limit.get();
  }

  public boolean getArmReverseLimit(){
      return m_arm_reverse_limit.get();
  }

  public boolean getWristForwardLimit(){
      return m_wrist_forward_limit.get();
  }

  public boolean getWristReverseLimit(){
      return m_wrist_reverse_limit.get();
  }

  public void drive_manually(double arm_speed, double wrist_speed) {
    m_arm_motor.set(ControlMode.PercentOutput, arm_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, wrist_speed);
  }

  public void drive_manually_by_position(double arm_speed, double wrist_speed){
    // Make sure we've got a number larger than 10%
    arm_speed = MathUtil.applyDeadband(arm_speed, .1);
    wrist_speed = MathUtil.applyDeadband(wrist_speed, .1);

    // Get the current encoder positions, and then calculate the future position
    // if we were to allow movement
    double arm_position = m_arm_motor.getSelectedSensorPosition();
    double wrist_position = m_wrist_motor.getSelectedSensorPosition();

    if( arm_speed != 0 ){
      int next_position = (int)( arm_position + ((arm_speed > 0) ? ARM_POSITION_STEP : ARM_POSITION_STEP * -1));
      // If it is within limits, then set the new position as the movement
      if( next_position < ARM_FORWARD_SOFT_LIMIT || next_position > ARM_REVERSE_SOFT_LIMIT) {
        arm_position = next_position;
      }
    }

    if( wrist_speed != 0){
      int next_position = (int)( wrist_position + ((arm_speed > 0) ? WRIST_POSITION_STEP : WRIST_POSITION_STEP * -1));
      // If it is within limits, then set the new position as the movement
      if( next_position < WRIST_FORWARD_SOFT_LIMIT || next_position > WRIST_REVERSE_SOFT_LIMIT) {
        wrist_position = next_position;
      }
    }

    // Set the arm and wrist motors with their proper PID slot, calculate the FeedForward, and set the motors
    double arm_ArbFF = 0;
    double wrist_ArbFF = 0;
    if( arm_speed > 0) {
      m_arm_motor.selectProfileSlot(ARM_MANUAL_FORWARD_SLOT, 0);
      arm_ArbFF = ARM_MANUAL_FORWARD_FF;
    } else if ( arm_speed < 0) {
      m_arm_motor.selectProfileSlot(ARM_MANUAL_REVERSE_SLOT, 0);
      arm_ArbFF = ARM_MANUAL_REVERSE_FF;
    } else {
      // Don't do anything
    }

    if( wrist_speed > 0) {
      m_wrist_motor.selectProfileSlot(WRIST_MANUAL_FORWARD_SLOT, 0);
      wrist_ArbFF = WRIST_MANUAL_FORWARD_FF;
    } else if ( arm_speed < 0) {
      m_wrist_motor.selectProfileSlot(WRIST_MANUAL_REVERSE_SLOT, 0);
    } else {
      // Don't do anything
    }

    m_arm_motor.set(ControlMode.Position, arm_position, DemandType.ArbitraryFeedForward, arm_ArbFF);
    m_wrist_motor.set(ControlMode.Position, wrist_position, DemandType.ArbitraryFeedForward, wrist_ArbFF);
  }

  ////////////////////// ARM INLINE COMMANDS  /////////////////////
  public CommandBase expandGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_EXPAND);
        });
  }
  
  public CommandBase contractGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_CONTRACT);
        });
  }

  public CommandBase setArmEncoderToZero() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_arm_motor.setSelectedSensorPosition(0);
        });
  }

  public CommandBase setWristEncoderToZero() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_wrist_motor.setSelectedSensorPosition(0);
        });
  }

  public CommandBase setArmMaxSpeed(){
    return runOnce(
      () -> {
        NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
        double forward_speed = driveTab.getEntry("Arm Fwd Spd").getDouble(ARM_FORWARD_SPEED);
        double reverse_speed = driveTab.getEntry("Arm Rev Spd").getDouble(ARM_REVERSE_SPEED);

        // Just in case they put values that aren't positive or negative on Shuffleboard as an accident, make sure it's right
        if( reverse_speed > 0 ){
          reverse_speed = reverse_speed * -1;
        }

        if( forward_speed < 0 ){
          forward_speed = forward_speed * -1;
        }

        m_arm_motor.configPeakOutputForward(forward_speed);
        m_arm_motor.configPeakOutputReverse(reverse_speed);
      });
  }

  public CommandBase setWristMaxSpeed(){
    return runOnce (
      () -> {
        NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm Test");
        double forward_speed = driveTab.getEntry("Wrist Fwd Spd").getDouble(WRIST_FORWARD_SPEED);
        double reverse_speed = driveTab.getEntry("Wrist Rev Spd").getDouble(WRIST_REVERSE_SPEED);

        // Just in case they put values that aren't positive or negative on Shuffleboard as an accident, make sure it's right
        if( reverse_speed > 0 ){
          reverse_speed = reverse_speed * -1;
        }

        if( forward_speed < 0 ){
          forward_speed = forward_speed * -1;
        }

        m_wrist_motor.configPeakOutputForward(forward_speed);
        m_wrist_motor.configPeakOutputReverse(reverse_speed);
    });
  }

  public CommandBase stopArmAndWristCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_arm_motor.set(ControlMode.PercentOutput, 0);
          m_wrist_motor.set(ControlMode.PercentOutput, 0);
        });
  }
}