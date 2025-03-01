// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
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

  /** Returns a Pose2d object representing the position of the robot in the field (to be used in addVisionMeasurement method).
   * 
   * Returns null if not targets are found. */
  /* public Pose2d getPose2D() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return null;
    PhotonTrackedTarget target = result.getBestTarget();
    return PhotonUtils.estimateFieldToRobot(CAMERA.CAMERA_HEIGHT)
  } */
}
