// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAMERA;

public class CameraSys extends SubsystemBase {

  private static PhotonCamera camera = new PhotonCamera(CAMERA.CAMERA_NICKNAME);
  
  /** Creates a new CameraSys. */
  public CameraSys() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Returns an EstimatedRobotPose object representing the position of the robot in the field (to be used in addVisionMeasurement method).
   * 
   * Returns null if not targets are found by the camera. */
  public EstimatedRobotPose getEstimatedRobotPose() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return null;
    if (CAMERA.PHOTON_POSE_ESTIMATOR.update(result).isPresent()) return CAMERA.PHOTON_POSE_ESTIMATOR.update(result).get();
    return null;
  }
}
