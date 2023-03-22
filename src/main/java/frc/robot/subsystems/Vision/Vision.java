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
  PhotonCamera m_limelight, m_lifecam;
  PhotonPipelineResult m_lime_result, m_life_result;
  PhotonTrackedTarget m_lime_target, m_life_target;

  double m_lime_tx, m_life_tx;
  boolean m_lime_has_targets, m_life_has_targets;

  public static final int LIME_CONE_PIPE = 0;
  public static final int LIME_APRILTAG_PIPE = 1;
  public static final int LIFE_CONE_PIPE = 0;
  public static final int LIFE_CUBE_PIPE = 1;
  public static final int LIFE_SUBSTATION_PIPE = 2;

  /** Creates a new Vision. */
  /*  Vision() {
    m_limelight = new PhotonCamera("limelight");
    m_lifecam = new PhotonCamera("lifecam");

    m_lime_result = new PhotonPipelineResult();
    m_life_result = new PhotonPipelineResult();

    m_lime_target = new PhotonTrackedTarget();
    m_life_target = new PhotonTrackedTarget();

    m_limelight.setPipelineIndex(LIME_CONE_PIPE);
    m_lifecam.setPipelineIndex(LIFE_CONE_PIPE);

    m_limelight.setLED(VisionLEDMode.kOff);

    m_lifecam.setDriverMode(true);
  }
  */

  @Override
  public void periodic() {
    resolvePhotonResults();
  }

  private void resolvePhotonResults() {
    m_lime_result = m_limelight.getLatestResult();
    m_life_result = m_lifecam.getLatestResult();

    m_lime_has_targets = m_lime_result.hasTargets();
    m_life_has_targets = m_life_result.hasTargets();

    if (m_lime_has_targets) {
      m_lime_target = m_lime_result.getBestTarget();
      m_lime_tx = m_lime_target.getYaw();
    }else {
      m_lime_tx = 0;
    }

    if (m_life_has_targets) {
      m_life_target = m_life_result.getBestTarget();
      m_life_tx = m_life_target.getYaw();
    }else {
      m_life_tx = 0;
    }
  }

  

  public boolean getLimelightTargetStatus() {
    return m_lime_has_targets;
  }

  public boolean getLifecamTargetStatus() {
    return m_life_has_targets;
  }

  public double getLimelightTX() {
    return m_lime_tx;
  }

  public double getLifecamTX() {
    return m_life_tx;
  }

  public void setLimelightLEDmode(boolean is_led_on) {
    if (is_led_on) {
      m_limelight.setLED(VisionLEDMode.kOn);
    }else {
      m_limelight.setLED(VisionLEDMode.kOff);
    }
  }

  public void setLimelightPipeline(int pipeline_index) {
    m_limelight.setPipelineIndex(pipeline_index);
  }

  public void setLifecamIndex(int pipeline_index) {
    m_lifecam.setPipelineIndex(pipeline_index);
  }
  
  public void setLifecamDriverMode(boolean driver_mode) {
    m_lifecam.setDriverMode(driver_mode);
  }

  public void setLimelightDriverMode(boolean driver_mode) {
    m_limelight.setDriverMode(driver_mode);
  }
}
