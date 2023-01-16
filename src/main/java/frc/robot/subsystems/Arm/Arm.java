// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX m_bottom_stage, m_top_stage;
  DigitalInput  m_arm_base_reverse_limit, m_arm_top_reverse_limit;
  DoublePublisher m_bottom_arm_enc_value, m_top_arm_enc_value;

  // Constant values for ARM movement, must be researched and tuned via tuner
  final double BOTTOM_ARM_DRIVE_SPEED = .1;
  final double TOP_ARM_DRIVE_SPEED = .1;
  /** Creates a new Arm. */

  public ArmState arm_state;
  public ArmPosition arm_position;
  public ArmTask arm_task;

  public enum ArmState {
    Active,
    Inactive,
  }

  public enum ArmPosition {
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
    Stop,
  }

  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive input rotates to FRONT of robot
    m_bottom_stage = new WPI_TalonFX(BOTTOM_ARM_STAGE);
    m_top_stage = new WPI_TalonFX(TOP_ARM_STAGE);
    
    TalonFXConfiguration bottom_configs = new TalonFXConfiguration();
    bottom_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_bottom_stage.configAllSettings(bottom_configs);
    TalonFXConfiguration top_config = new TalonFXConfiguration();
    top_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_top_stage.configAllSettings(top_config);
    
    // Top and bottom arm limit switches
    m_arm_base_reverse_limit = new DigitalInput(ARM_BASE_LIMIT_SWITCH);
    m_arm_top_reverse_limit = new DigitalInput(ARM_TOP_LIMIT_SWITCH);

    // Setup the network tables publishers to push data to the dashboard
    NetworkTable shuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard");
    m_bottom_arm_enc_value = shuffleboard.getDoubleTopic("Arm/BottomEncoder").publish();
    m_top_arm_enc_value = shuffleboard.getDoubleTopic("Arm/TopEncoder").publish();

    arm_state = ArmState.Inactive;
    arm_position = ArmPosition.Retracted;
    arm_task = ArmTask.Stop;
  }

  @Override
  public void periodic() {
    m_bottom_arm_enc_value.set(m_bottom_stage.getSelectedSensorPosition());
    m_top_arm_enc_value.set(m_top_stage.getSelectedSensorPosition());
  }

  public void moveArmManually(double bottom_speed, double top_speed) {
    m_bottom_stage.set(ControlMode.PercentOutput, bottom_speed);
    m_top_stage.set(ControlMode.PercentOutput, top_speed);

    if (m_arm_base_reverse_limit.get() && m_bottom_stage.getSelectedSensorVelocity() < 0) {
      m_bottom_stage.set(ControlMode.PercentOutput, 0);
    }

    if (m_arm_top_reverse_limit.get() && m_top_stage.getSelectedSensorVelocity() < 0) {
      m_top_stage.set(ControlMode.PercentOutput, 0);
    }
  }

  public void armFullStop() {
    arm_task = ArmTask.Stop;
    m_bottom_stage.set(ControlMode.PercentOutput, 0);
    m_top_stage.set(ControlMode.PercentOutput, 0);
  }

  // DO NOT USE DURING MATCHES!!! ONLY RUN IN THE PITS!!!
  public void reset_arm_pos() {
    // while BOTH top and bottoms are false (not triggered)
    while (!m_arm_base_reverse_limit.get() && !m_arm_top_reverse_limit.get()) {
      // only drive base if limit is not hit
      if (!m_arm_base_reverse_limit.get()) {
        m_bottom_stage.set(ControlMode.PercentOutput, -BOTTOM_ARM_DRIVE_SPEED);
      }else {
        m_bottom_stage.set(ControlMode.PercentOutput, 0);
      }

      // only drive upper stage if limit is not hit
      if (!m_arm_top_reverse_limit.get()) {
        m_top_stage.set(ControlMode.PercentOutput, -TOP_ARM_DRIVE_SPEED);
      }else {
        m_top_stage.set(ControlMode.PercentOutput, 0);
      }
    }
    
    // Zero encoders once both stages are reset
    m_bottom_stage.setSelectedSensorPosition(0);
    m_top_stage.setSelectedSensorPosition(0);
  }

  public WPI_TalonFX getBottomStageMotor() {
    return m_bottom_stage;
  }

  public WPI_TalonFX getTopStageMotor() {
    return m_top_stage;
  }

  public ArmState getArmState() {
    return arm_state;
  }

  public ArmPosition getArmPosition() {
    return arm_position;
  }

  public ArmTask getArmTask() {
    return arm_task;
  }
}