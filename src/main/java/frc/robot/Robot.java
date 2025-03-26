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
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private boolean isAutoStarted = false;
   

  private final AutoChooser autoChooser;

  public Robot() {
    m_robotContainer = new RobotContainer();
    autoChooser = new AutoChooser();

    //elia, use .addRoutine() to do things
    //autoChooser.addRoutine(null, null);
    autoChooser.addCmd("dumboBlueRightAutoRoutine", ()->m_robotContainer.auto.dumboBlueRightAutoRoutine1Command());
    autoChooser.addRoutine("eliatestroutine", () -> m_robotContainer.auto.getTestRoutine());
//whr
    
    //Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooser);
    
    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

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
    //m_robotContainer.swerveDriveManager.setStartPose();;

    m_autonomousCommand = m_robotContainer.auto.scoreTestElevatorCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.disabledAuto();
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
