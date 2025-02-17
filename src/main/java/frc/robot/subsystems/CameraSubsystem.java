// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Arrays;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CameraConstants.*;

public class CameraSubsystem extends SubsystemBase {
  

  AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(kApriltags, kFieldLenght.magnitude(), kFieldWidth.magnitude());
  
  PhotonCamera mainCamera = new PhotonCamera("Arducam");
  List<PhotonPipelineResult> results = null;

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);


  public boolean hasTarget = false;
  public double targetYaw = 0;
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = mainCamera.getAllUnreadResults();
    processResults();
    
  }
  public void processResults()
  {
    //Check if we have a frame
    if (results.isEmpty())
    {
      hasTarget = false;
      return;
    }
    //Check if we have a target
    if(!getBestResult().hasTargets())
    {
      hasTarget = false;
      return;
    }
    
    var target = getBestResult().getBestTarget();
    hasTarget = true;
    if(target != null)
       targetYaw = target.getYaw();
  }
  
 public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        
        if(!results.isEmpty())
        {
          if(getBestResult().hasTargets())
          {
            visionEst = photonPoseEstimator.update(getBestResult());
          }

        }
        return visionEst;
  }




public PhotonPipelineResult getBestResult(){
  return results.get(results.size() - 1);
}

}