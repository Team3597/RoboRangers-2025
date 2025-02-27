// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSys extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera frontCam = new PhotonCamera("Front Orange Pi Cam");
  

  public VisionSys() {}

  public PhotonTrackedTarget getTag(int id) {
    var result = frontCam.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget desiredTarget = null;
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == id) {
          desiredTarget = target;
        }
      }
      if (desiredTarget != null) {
        return desiredTarget;
      } else {
        return null;
      }
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
