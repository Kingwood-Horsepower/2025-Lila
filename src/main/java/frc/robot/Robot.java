// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    CommandScheduler.getInstance()
    .onCommandInitialize(
        command ->
            // Shuffleboard.addEventMarker(
            //     "Command initialized", command.getName(), EventImportance.kNormal));
            SmartDashboard.putString("commands", "initialize " + command.getName()));
  CommandScheduler.getInstance()
    .onCommandFinish(
        command ->
            // Shuffleboard.addEventMarker(
            //     "Command finished", command.getName(), EventImportance.kNormal));
            SmartDashboard.putString("commands", "end command " + command.getName()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.resetPose();

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
    m_robotContainer.disableAuto();
  }

  @Override
  public void teleopInit() {
    System.out.println(RobotContainer.camera.getCoralScoreTransform(8, false));
    System.out.println(RobotContainer.camera.getCoralScoreTransform(8, true));
    System.out.println(RobotContainer.camera.getStationPose2d(12));
    m_robotContainer.resetPose();
    
  }

  @Override
  public void teleopPeriodic() 
  {
    if(m_robotContainer != null){
      m_robotContainer.UpdateRobotPosition();
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    
  }
}
