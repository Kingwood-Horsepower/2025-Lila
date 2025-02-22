// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import java.util.Arrays;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
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
  

  //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(kApriltags, kFieldLenght.magnitude(), kFieldWidth.magnitude());
  
  PhotonCamera rightCamera = new PhotonCamera("RightCam");
  PhotonCamera leftCamera = new PhotonCamera("LeftCam");

  List<PhotonPipelineResult> rightResults = null;
  List<PhotonPipelineResult> leftResults = null;
  
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToRightCam);
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToLeftCam);

  double ambiguityRight;
  boolean hasTargetRight = false;
  double targetYawRight = 0;
  double targetRangeRight = 0;

  double ambiguityLeft;
  boolean hasTargetLeft = false;
  double targetYawLeft = 0;
  double targetRangeLeft = 0;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightResults = rightCamera.getAllUnreadResults();
    leftResults = leftCamera.getAllUnreadResults();
    processRightResults();
    processLeftResults();
  }
  public boolean processRightResults()
  {
    //Check if we have a frame
    if (rightResults.isEmpty())
    {
      hasTargetRight = false;
      return false;
    }
    //Check if we have a target
    if(!getBestRightResult().hasTargets())
    {
      hasTargetRight = false;
      return false;
    }
    
    var target = getBestRightResult().getBestTarget();
    hasTargetRight = true;
    if(target != null){
      ambiguityRight = target.poseAmbiguity;
      targetYawRight = target.getYaw();
      targetRangeRight = PhotonUtils.calculateDistanceToTargetMeters(kRobotToRightCam.getZ(), aprilTagFieldLayout.getTagPose(target.fiducialId).get().getRotation().getY(), kRobotToRightCam.getRotation().getY(), Math.toRadians((target.getPitch())));
      return true;
    }
    return false;
  }
  public boolean processLeftResults()
  {
    //Check if we have a frame
    if (leftResults.isEmpty())
    {
      hasTargetLeft = false;
      return false;
    }
    //Check if we have a target
    if(!getBestLeftResult().hasTargets())
    {
      hasTargetLeft = false;
      return false;
    }
    
    var target = getBestLeftResult().getBestTarget();
    hasTargetLeft= true;
    if(target != null){
      ambiguityLeft = target.poseAmbiguity;
      targetYawLeft = target.getYaw();
      targetRangeLeft = PhotonUtils.calculateDistanceToTargetMeters(kRobotToRightCam.getZ(), aprilTagFieldLayout.getTagPose(target.fiducialId).get().getRotation().getY(), kRobotToRightCam.getRotation().getY(), Math.toRadians((target.getPitch())));
      return true;
    }
    return false;
  }
 public Optional<EstimatedRobotPose> getEstimatedGlobalRightPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        if(!rightResults.isEmpty())
        {
          if(getBestRightResult().hasTargets())
          {
            visionEst = rightPoseEstimator.update(getBestRightResult());
          }

        }
        return visionEst;
  }
  public Optional<EstimatedRobotPose> getEstimatedGlobalLeftPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    
    if(!leftResults.isEmpty())
    {
      if(getBestLeftResult().hasTargets())
      {
        visionEst = leftPoseEstimator.update(getBestLeftResult());
      }

    }
    return visionEst;
}




PhotonPipelineResult getBestRightResult(){
  return rightResults.get(rightResults.size() - 1);
}

PhotonPipelineResult getBestLeftResult(){
  return leftResults.get(leftResults.size() - 1);
}

public boolean hasTarget(){
  return hasTargetLeft || hasTargetRight;
}
public double getTargetYaw(){
    //Chose one with lowest ambiguity
    if(hasTargetLeft && hasTargetRight)
    if(ambiguityLeft < ambiguityRight){
      return targetYawLeft;
    }else 
    return targetYawRight;
 //if only one actually has a target
  if(hasTargetLeft)
    return targetYawLeft;
  else
    return targetYawRight;
}
public double getTargetRange(){
  //Chose one with lowest ambiguity
  if(hasTargetLeft && hasTargetRight)
    if(ambiguityLeft < ambiguityRight){
      return targetRangeLeft;
    }else 
    return targetRangeRight;
 //if only one actually has a target
  if(hasTargetLeft)
    return targetRangeLeft;
  else 
    return targetRangeRight;
}

}