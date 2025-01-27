// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera mainCamera = new PhotonCamera("Arducam");
  List<PhotonPipelineResult> results = null;

  public boolean isTarget = false;
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
      isTarget = false;
      return;
    }
    var result = results.get(results.size() - 1);

    //Check if we have a target
    if(!result.hasTargets())
    {
      isTarget = false;
      return;
    }
    
    var target = result.getBestTarget();
    isTarget = true;
    
    targetYaw = target.getYaw();
  }
  
}
