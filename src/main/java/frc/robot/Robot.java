// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;

  private boolean isAutoStarted = false;
   

  public Robot() {
    m_robotContainer = new RobotContainer();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    if(m_robotContainer != null){
      m_robotContainer.UpdateRobotPosition();
    }
    //m_
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    isAutoStarted = true;
    //Enable this for auto testing. Don't forget to comment the autoroutine.poll in Auto
    // m_robotContainer.auto.scoreTestElevatorCommand().schedule;
  }

  @Override
  public void autonomousPeriodic() 
  {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    if(!isAutoStarted){
      m_robotContainer.swerveDriveManager.setStartPose();
    }
    isAutoStarted = false;
    m_robotContainer.stateMachine.startStateMachine();
    //m_robotContainer.loadPreferences();
  
  }

  @Override
  public void teleopPeriodic() 
  {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.stateMachine.startStateMachineTest();
    
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    //m_robotContainer.visionManager.printScoringPosition();
  }
}
