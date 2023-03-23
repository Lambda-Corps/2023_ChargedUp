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
  PhotonCamera m_limelight;
  PhotonPipelineResult m_lime_result;
  PhotonTrackedTarget m_lime_target;

  double m_lime_tx;
  boolean m_lime_has_targets;

  public static final int LIME_CONE_PIPE = 0;
  public static final int LIME_APRILTAG_PIPE = 1;

  /** Creates a new Vision. */
  /*  Vision() {
    m_limelight = new PhotonCamera("limelight");
    m_lime_result = new PhotonPipelineResult();
    m_lime_target = new PhotonTrackedTarget();
    m_limelight.setPipelineIndex(LIME_CONE_PIPE);
    m_limelight.setLED(VisionLEDMode.kOff);
  }
  */

  @Override
  public void periodic() {
    resolvePhotonResults();
  }

  private void resolvePhotonResults() {
    m_lime_result = m_limelight.getLatestResult();

    m_lime_has_targets = m_lime_result.hasTargets();

    if (m_lime_has_targets) {
      m_lime_target = m_lime_result.getBestTarget();
      m_lime_tx = m_lime_target.getYaw();
    }else {
      m_lime_tx = 0;
    }
  }

  

  public boolean getLimelightTargetStatus() {
    return m_lime_has_targets;
  }

  public double getLimelightTX() {
    return m_lime_tx;
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

  public void setLimelightDriverMode(boolean driver_mode) {
    m_limelight.setDriverMode(driver_mode);
  }
}
