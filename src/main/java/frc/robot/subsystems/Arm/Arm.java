// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX m_arm_motor, m_wrist_motor;
  DigitalInput  m_arm_forward_limit, m_arm_reverse_limit, m_wrist_reverse_limit, m_wrist_forward_limit;
  DoublePublisher m_arm_position, m_wrist_position;

  /** Creates a new Arm. */

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
    Manual_Control,
    Retracted,
    GroundPickup,
    SubstationPickup,
    ConeScoreLow,
    ConeScoreMid,
    ConeScoreHigh,
    CubeScoreLow,
    CubeScoreMid,
    CubeScoreHigh,
  }

  public enum ArmTask {
    HoldPosition,
    MoveToPosition,
    MoveManually,
    Stop,
  }

  ///////// Constants ///////////////
  // Constant values for ARM movement, must be researched and tuned via tuner
  final double ARM_DRIVE_SPEED = .2;
  final double WRIST_DRIVE_SPEED = .2;
  final double ARM_GEAR_RATIO = 10 * 4 * 4;
  final double WRIST_GEAR_RATIO = 10 * 7 * 7;
  final int WRIST_REVERSE_SOFT_LIMIT = 32000;
  final int WRIST_FORWARD_SOFT_LIMIT = 150000;
  final int ARM_REVERSE_SOFT_LIMIT = 10000; // TODO TUNE THIS
  final int ARM_FORWARD_SOFT_LIMIT = (int)(2048 * ARM_GEAR_RATIO * 1/6); // 60 degrees rotation
  final double WRIST_MAX_STATOR_CURRENT = 12.5;
  final double ARM_MAX_STATOR_CURRENT = 5;
  final int MOTION_MAGIC_SLOT = 0;


  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive input rotates to FRONT of robot
    m_arm_motor = new WPI_TalonFX(ARM_MOTOR);
    m_wrist_motor = new WPI_TalonFX(WRIST_MOTOR);

    // Factory default the talons
    m_arm_motor.configFactoryDefault();
    m_wrist_motor.configFactoryDefault();
    
    TalonFXConfiguration arm_config = new TalonFXConfiguration();
    TalonFXConfiguration wrist_config = new TalonFXConfiguration();

    // Setup the ARM stage motor
    m_arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, MOTION_MAGIC_SLOT, 0);
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
    arm_config.peakOutputForward = ARM_DRIVE_SPEED;
    arm_config.peakOutputReverse = -ARM_DRIVE_SPEED;

    // Configure the arm
    m_arm_motor.configAllSettings(arm_config);
   
    // Configure the Wrist motor
    m_wrist_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, MOTION_MAGIC_SLOT, 0);
    wrist_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_wrist_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_wrist_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Configure limit switches
    wrist_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    wrist_config.reverseSoftLimitEnable = true;
    wrist_config.forwardSoftLimitEnable = true;
    wrist_config.reverseSoftLimitThreshold = ARM_REVERSE_SOFT_LIMIT;
    wrist_config.forwardSoftLimitThreshold = ARM_FORWARD_SOFT_LIMIT;

    // Set current limits for the ARM
    stator_limit = wrist_config.statorCurrLimit;
    stator_limit.currentLimit = ARM_MAX_STATOR_CURRENT;
    stator_limit.enable = true;
    stator_limit.triggerThresholdCurrent = ARM_MAX_STATOR_CURRENT + 1;
    wrist_config.statorCurrLimit = stator_limit;
    
    // Set max speeds for output
    wrist_config.peakOutputForward = ARM_DRIVE_SPEED;
    wrist_config.peakOutputReverse = -ARM_DRIVE_SPEED;
    // Configure the arm
    m_wrist_motor.configAllSettings(arm_config);
    m_wrist_motor.setInverted(TalonFXInvertType.Clockwise);

    // Top and bottom arm limit switches, these DIOs are for the LEDs to light up when the limits are hit
    m_arm_forward_limit = new DigitalInput(ARM_FORWARD_LIMIT_SWITCH);
    m_arm_reverse_limit = new DigitalInput(ARM_REVERSE_LIMIT_SWITCH);
    
    m_wrist_forward_limit = new DigitalInput(WRIST_FORWARD_LIMIT_SWITCH);
    m_wrist_reverse_limit = new DigitalInput(WRIST_REVERSE_LIMIT_SWITCH);

    // Setup the network tables publishers to push data to the dashboard
    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ARM");
    m_arm_position = shuffleboard.getDoubleTopic("Armposition").publish();
    m_wrist_position = shuffleboard.getDoubleTopic("Wristposition").publish();

    arm_state = ArmState.Inactive;
    arm_control_mode = ArmControlMode.Automatic;
    arm_position = ArmPosition.Retracted;
    arm_task = ArmTask.Stop;
  }

  @Override
  public void periodic() {
    m_arm_position.set(m_arm_motor.getSelectedSensorPosition());
    m_wrist_position.set(m_wrist_motor.getSelectedSensorPosition());
  }

  public void moveArmManually(double bottom_speed, double top_speed) {
    arm_state = ArmState.Active;
    arm_control_mode = ArmControlMode.Manual;
    arm_position = ArmPosition.Manual_Control;
    arm_task = ArmTask.MoveManually;

    m_arm_motor.set(ControlMode.PercentOutput, bottom_speed);
    m_wrist_motor.set(ControlMode.PercentOutput, top_speed);

    if (m_arm_forward_limit.get() && bottom_speed < 0) {
      m_arm_motor.set(ControlMode.PercentOutput, 0);
    }

    if (m_arm_reverse_limit.get() && top_speed < 0) {
      m_wrist_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void armFullStop() {
    arm_state = ArmState.Inactive;
    arm_control_mode = ArmControlMode.Automatic;
    arm_position = ArmPosition.Manual_Control;
    arm_task = ArmTask.Stop;

    m_arm_motor.set(ControlMode.PercentOutput, 0);
    m_wrist_motor.set(ControlMode.PercentOutput, 0);
  }

  // DO NOT USE DURING MATCHES!!! ONLY RUN IN THE PITS!!!
  public void reset_arm_pos() {
    arm_control_mode = ArmControlMode.Automatic;
    // while BOTH top and bottoms are false (not triggered)
    while (!m_arm_forward_limit.get() && !m_arm_reverse_limit.get()) {
      // only drive base if limit is not hit
      if (!m_arm_forward_limit.get()) {
        m_arm_motor.set(ControlMode.PercentOutput, -ARM_DRIVE_SPEED);
      }else {
        m_arm_motor.set(ControlMode.PercentOutput, 0);
      }

      // only drive upper stage if limit is not hit
      if (!m_arm_reverse_limit.get()) {
        m_wrist_motor.set(ControlMode.PercentOutput, -WRIST_DRIVE_SPEED);
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

  
}