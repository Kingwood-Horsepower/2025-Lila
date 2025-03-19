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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CameraConstants;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CameraConstants.*;

public class CameraSubsystem extends SubsystemBase {
  

  //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(kApriltags, kFieldLenght.magnitude(), kFieldWidth.magnitude());
  
  PhotonCamera rightCamera = new PhotonCamera("RightCam");
  PhotonCamera leftCamera = new PhotonCamera("LeftCam");
  PhotonCamera upCamera = new PhotonCamera("UpCam");

  List<PhotonPipelineResult> rightResults = null;
  List<PhotonPipelineResult> leftResults = null;
  List<PhotonPipelineResult> upResults = null;
  
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToRightCam);
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToLeftCam);
  PhotonPoseEstimator upPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToUpCam);

  double ambiguityRight;
  boolean hasTargetRight = false;
  PhotonTrackedTarget rightTarget;
  double targetRangeRight = 0;

  double ambiguityLeft;
  boolean hasTargetLeft = false;
  PhotonTrackedTarget leftTarget;
  double targetRangeLeft = 0;

  double ambiguityUp;
  boolean hasTargetUp = false;
  PhotonTrackedTarget upTarget;
  double targetRangeUp = 0;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightResults = rightCamera.getAllUnreadResults();
    leftResults = leftCamera.getAllUnreadResults();
    upResults = upCamera.getAllUnreadResults();

    processRightResults();
    processLeftResults();
    processUpResults();
  }
  private boolean processRightResults()
  {
    //Check if we have a frame
    if (rightResults.isEmpty())
    {
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
  private boolean processLeftResults()
  {
    //Check if we have a frame
    if (leftResults.isEmpty())
    {
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
  private boolean processUpResults()
  {
    //Check if we have a frame
    if (upResults.isEmpty())
    {
      return false;
    }
    //Check if we have a target
    if(!getBestUpResult().hasTargets())
    {
      hasTargetUp = false;
      return false;
    }
    
    var target = getBestUpResult().getBestTarget();
    hasTargetUp = true;
    if(target != null){
      ambiguityUp = target.poseAmbiguity;
      upTarget = target;
      targetRangeUp = PhotonUtils.calculateDistanceToTargetMeters(kRobotToUpCam.getZ(), aprilTagFieldLayout.getTagPose(target.fiducialId).get().getRotation().getY(), kRobotToUpCam.getRotation().getY(), Math.toRadians((target.getPitch())));
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
public Optional<EstimatedRobotPose> getEstimatedGlobalUpPose() {
  Optional<EstimatedRobotPose> visionEst = Optional.empty();
  
  if(!upResults.isEmpty())
  {
    if(getBestUpResult().hasTargets())
    {
      visionEst = upPoseEstimator.update(getBestUpResult());
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

PhotonPipelineResult getBestUpResult(){
  return upResults.get(upResults.size() - 1);
}

public PhotonTrackedTarget getBestDownTarget(){
  if(hasTargetLeft && hasTargetRight) {
    if(ambiguityLeft < ambiguityRight) return leftTarget;
    else return rightTarget;
  } else if (hasTargetLeft) return leftTarget;
  else return rightTarget;
}

public PhotonTrackedTarget getBestUpTarget(){
  return upTarget;
}

public boolean hasUpTarget(){
  return hasTargetUp;
}

public boolean hasDownTarget(){
  return hasTargetLeft || hasTargetRight;
}

public double getDownTargetRange(){
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

public Pose2d getCoralScoreTransform(int AprilTagId, boolean getRightCoral){
  boolean isBlue = AprilTagId > 15;
  Translation2d reefCenter = isBlue ? kBlueReefCenter : kRedReefCenter;
  
  //Vector from the center to the april tag
  Translation2d v = aprilTagFieldLayout.getTagPose(AprilTagId).get().getTranslation().toTranslation2d().minus(reefCenter);
  double vMangnitude = v.getNorm();
  Translation2d vNormalized = new Translation2d(v.getX()/vMangnitude, v.getY()/vMangnitude);
  v = vNormalized.times(vMangnitude + kDistanceFromApriltagWhenScoring);

  //CounterClockwise
  Translation2d vPerpendicular = new Translation2d(-vNormalized.getY(), vNormalized.getX());

  if(getRightCoral)
    return new Pose2d(reefCenter.plus(v.plus(vPerpendicular.times(kDistanceFromCoralToAprilTag+kRobotToCoralIntakeLeftOffset))), 
    new Rotation2d(aprilTagFieldLayout.getTagPose(AprilTagId).get().getRotation().getZ() + Math.PI));
  else 
    return new Pose2d(reefCenter.plus(v.minus(vPerpendicular.times(kDistanceFromCoralToAprilTag-kRobotToCoralIntakeLeftOffset+inchesToMeters(0.5)))), 
    new Rotation2d(aprilTagFieldLayout.getTagPose(AprilTagId).get().getRotation().getZ() + Math.PI));

  }

//DOES NOT WORK because top camera does not align with the reef
public Pose2d getStationPose2d(int AprilTagId){
  //int id =  getStationAprilTagId(AprilTagId);

  Translation2d aprilTagTranslation2d =  aprilTagFieldLayout.getTagPose(AprilTagId).get().getTranslation().toTranslation2d();
  double rotation = 0;
  
  switch (AprilTagId) {
    case 12:
      rotation = Math.toRadians(234);
      break;
    case 13:
      rotation = Math.toRadians(126);
      break;
    case 1:
      rotation = Math.toRadians(126 + 180);
      break;
    case 2:
      rotation = Math.toRadians(234 - 180);
      break;
    default:
      System.out.println("Invalid AprilTagId");
      break;
  }
  
  Translation2d fromAprilTagToRobot = new Translation2d(Math.cos(rotation), Math.sin(rotation));

  return new Pose2d(aprilTagTranslation2d.minus(fromAprilTagToRobot.times(kDistanceFromStationTorRobot)), new Rotation2d(rotation));
}
}

