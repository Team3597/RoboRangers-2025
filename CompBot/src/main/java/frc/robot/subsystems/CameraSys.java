// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAMERA;
import frc.robot.Constants.GLOBAL;

public class CameraSys extends SubsystemBase {

  private static PhotonCamera camera = new PhotonCamera(CAMERA.CAMERA_NICKNAME); // camera is constant (not sure if this should be final?) so no need for initialization in constructor

  /** Creates a new CameraSys. */
  public CameraSys() {
    // if (GLOBAL.DEBUG_MODE) System.out.println("in camera constructor");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Returns an EstimatedRobotPose object representing the position of the robot in the field (to be used in addVisionMeasurement method).
   * 
   * Returns null if not targets are found by the camera (check for null to avoid a nullPointerException error). */
  public EstimatedRobotPose getEstimatedRobotPose() {
    PhotonPipelineResult result = camera.getLatestResult(); // gets result from the camera
    if (GLOBAL.DEBUG_MODE) System.out.println("got results from camera");
    if (!result.hasTargets()) return null; // checks if result has targets
    // if (CAMERA.PHOTON_POSE_ESTIMATOR.update(result).isPresent()) 
    //   return CAMERA.PHOTON_POSE_ESTIMATOR.update(result); // returns an EstimatedRobotPose if available from targets
    return null; // returns null otherwise
  }

  public void getAprilID() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) System.out.println(result.getBestTarget().getFiducialId());
    else System.out.println(false);
  }
}
