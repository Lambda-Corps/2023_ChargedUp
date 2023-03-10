// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera m_limelight, m_lifecam;

  double m_lime_cur_pipeline, m_life_cur_pipeline;

  // pipeline results
  PhotonPipelineResult m_limelight_result, m_lifecam_result;

  // tracked targets
  PhotonTrackedTarget m_limelight_target, m_lifecam_target;

  // target properties
  double m_lime_pitch, m_lime_yaw, m_lime_area, m_life_pitch, m_life_yaw, m_life_area;

  public Vision() {
    m_limelight = new PhotonCamera("limelight");
    m_lifecam = new PhotonCamera("lifecam");

    // inital limelight configs
    m_limelight.setPipelineIndex(0);
    m_limelight.setLED(VisionLEDMode.kOff);
    m_limelight.setDriverMode(false);

    // initial lifecam configs
    m_lifecam.setPipelineIndex(0);
    m_lifecam.setDriverMode(true);
  }

  @Override
  public void periodic() {
    // Gather pipeline results every scheduler run so that subsequent processing happens to the same result
    m_limelight_result = m_limelight.getLatestResult();
    m_lifecam_result = m_lifecam.getLatestResult();
    process_latest_results();
  }

  private void process_latest_results() {
    if (m_limelight_result.hasTargets()) {
      m_limelight_target = m_limelight_result.getBestTarget();

      m_lime_pitch = m_limelight_target.getPitch();
      m_lime_yaw = m_limelight_target.getYaw();
      m_lime_area = m_limelight_target.getArea();
    }else {
      m_lime_pitch = 0;
      m_lime_yaw = 0;
      m_lime_area = 0;
    }

    if (m_lifecam_result.hasTargets()) {
      m_lifecam_target = m_lifecam_result.getBestTarget();

      m_life_pitch = m_lifecam_target.getPitch();
      m_life_yaw = m_lifecam_target.getYaw();
      m_life_area = m_lifecam_target.getArea();
    }else {
      m_life_pitch = 0;
      m_life_yaw = 0;
      m_life_area = 0;
    }
  }

  public double get_limelight_target_pitch() {
    return m_lime_pitch;
  }
  
  public double get_limelight_target_yaw() {
    return m_lime_yaw;
  }
  public double get_limelight_target_area() {
    return m_lime_area;
  }

  public double get_lifecam_target_pitch() {
    return m_life_pitch;
  }
  
  public double get_lifecam_target_yaw() {
    return m_life_yaw;
  }
  public double get_lifecam_target_area() {
    return m_life_area;
  }

  public void turn_on_limelight_LEDs() {
    m_limelight.setLED(VisionLEDMode.kOn);
  }

  public void turn_off_limelight_LEDs() {
    m_limelight.setLED(VisionLEDMode.kOff);
  }

  public void set_limelight_pipeline_index(int pipeline_index) {
    m_limelight.setPipelineIndex(pipeline_index);
  }

  public void set_lifecam_pipeline_index(int pipeline_index) {
    m_lifecam.setPipelineIndex(pipeline_index);
  }

  public void set_lifecam_driver_mode(boolean is_pipeline_in_driver_mode) {
    m_lifecam.setDriverMode(is_pipeline_in_driver_mode);
  }
}
