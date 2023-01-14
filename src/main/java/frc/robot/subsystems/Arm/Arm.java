// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  TalonFXInvertType bottom_invert_type, top_invert_type;
  NeutralMode bottom_neutral_mode, top_neutral_mode;
  WPI_TalonFX m_bottom_stage, m_top_stage;
  TalonFXConfiguration bottom_configs, top_configs;
  DigitalInput  m_arm_base_reverse_limit, m_arm_top_reverse_limit;

  final double BOTTOM_ARM_DRIVE_SPEED, TOP_ARM_DRIVE_SPEED;
  /** Creates a new Arm. */
  public Arm() {
    // Configure top and bottom arm talons. NOTE: set invertType such that positive input rotates to FRONT of robot
    m_bottom_stage = new WPI_TalonFX(BOTTOM_ARM_STAGE);
    bottom_invert_type = TalonFXInvertType.CounterClockwise;
    bottom_neutral_mode = NeutralMode.Brake;
    m_top_stage = new WPI_TalonFX(TOP_ARM_STAGE);
    top_invert_type = TalonFXInvertType.CounterClockwise;
    top_neutral_mode = NeutralMode.Brake;

    bottom_configs = new TalonFXConfiguration();
    bottom_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_bottom_stage.configAllSettings(bottom_configs);
    top_configs = new TalonFXConfiguration();
    top_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_top_stage.configAllSettings(top_configs);
    
    m_bottom_stage.setInverted(bottom_invert_type);
    m_bottom_stage.setNeutralMode(bottom_neutral_mode);
    m_top_stage.setInverted(top_invert_type);
    m_top_stage.setNeutralMode(top_neutral_mode);

    // Top and bottom arm limit switches
    m_arm_base_reverse_limit = new DigitalInput(ARM_BASE_LIMIT_SWITCH);
    m_arm_top_reverse_limit = new DigitalInput(ARM_TOP_LIMIT_SWITCH);

    ////////////// ARM CONSTANTS //////////////
    BOTTOM_ARM_DRIVE_SPEED = 1.0; /* subject to change after testing */
    TOP_ARM_DRIVE_SPEED = 1.0; /* subject to change after testing */
  }

  @Override
  public void periodic() {
    
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
}