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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CameraConstants;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CameraConstants.*;

public class CameraSubsystem extends SubsystemBase {
  

  //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(kApriltags, kFieldLenght.magnitude(), kFieldWidth.magnitude());
  
  PhotonCamera rightCamera = new PhotonCamera("RightCam");
  PhotonCamera leftCamera = new PhotonCamera("LeftCam");

  List<PhotonPipelineResult> rightResults = null;
  List<PhotonPipelineResult> leftResults = null;
  
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToRightCam);
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToLeftCam);

  double ambiguityRight;
  boolean hasTargetRight = false;
  PhotonTrackedTarget rightTarget;
  double targetRangeRight = 0;

  double ambiguityLeft;
  boolean hasTargetLeft = false;
  PhotonTrackedTarget leftTarget;
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
      rightTarget= target;
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
      leftTarget = target;
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

public PhotonTrackedTarget getBestTarget(){
  if(hasTargetLeft && hasTargetRight) {
    if(ambiguityLeft < ambiguityRight) return leftTarget;
    else return rightTarget;
  } else if (hasTargetLeft) return leftTarget;
  else return rightTarget;
}

public boolean hasTarget(){
  return hasTargetLeft || hasTargetRight;
}
public double getTargetSkew(){
    return getBestTarget().getSkew();
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
public Translation2d getCoralScoreTransform(int AprilTagId, boolean getRightCoral){
  int id =  getAprilTagId(AprilTagId);

  //Vector from the center to the april tag
  Translation2d v = aprilTagFieldLayout.getTagPose(id).get().getTranslation().toTranslation2d().minus(kReefCenter);
  double vMangnitude = v.getNorm();
  Translation2d vNormalized = new Translation2d(v.getX()/vMangnitude, v.getY()/vMangnitude);
  v = vNormalized.times(vMangnitude + kDistanceFromApriltagWhenScoring);

  //CounterClockwise
  Translation2d vPerpendicular = new Translation2d(-vNormalized.getY(), vNormalized.getX());

  if(getRightCoral)
   return kReefCenter.plus(v.plus(vPerpendicular.times(kDistanceFromCoralToAprilTag)));
  else 
  return kReefCenter.plus(v.minus(vPerpendicular.times(kDistanceFromCoralToAprilTag)));

}
public int getAprilTagId(int id){
  //if the id is in the blue allience, return its id
  if(id> 15)
    return id;
  //if the id is in the red alliance, convert it to its blue allience equivalent
  else 
  {
    switch (id){
      case 7: return 18;
      case 8: return 17;
      case 9: return 22;
      case 10: return 21;
      case 11: return 20;
      case 12: return 19;
      default:
      System.err.println("Wrong April Tag");
       return -1;
    }

      
  }
}

}