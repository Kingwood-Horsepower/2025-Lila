package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants.TargetCoralStation;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.AutoConstants.*;

public class Auto {
    private final AutoFactory autoFactory;
    private final RobotContainer robotContainer;

    private final AutoRoutine autoRoutine;
    private final SwerveDriveManager swerveDriveManager;

    public Auto(SwerveDriveManager swerveDriveManager, RobotContainer _robotContainer)
    {   
        autoFactory = new AutoFactory(
            swerveDriveManager::getRobotPose, // A function that returns the current robot pose
            swerveDriveManager::resetPose, // A function that resets the current robot pose to the provided Pose2d
            swerveDriveManager::followTrajectory, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            swerveDriveManager.getDrivetrain() // The drive subsystem
        );

        this.swerveDriveManager = swerveDriveManager;
        robotContainer = _robotContainer;
        //autoRoutine = getAutoRoutine();
        autoRoutine = getTestRoutine();

        
    }
    public void PollAutoRoutine()
    {
        autoRoutine.poll();
    }
    public void KillAutoRoutine()
    {
        autoRoutine.kill();
    }
    
    @SuppressWarnings("unused")
    AutoRoutine getAutoRoutine()
    {
        AutoRoutine routine = autoFactory.newRoutine("Autonomous");
        AutoTrajectory goToCoralStation = getStartingAutoTrajectory(routine);

        //custom starting trajectory based on the starting position and the targetStation
       routine.active().onTrue(
        Commands.sequence(
            goToCoralStation.resetOdometry(),
            goToCoralStation.cmd()
        )
        );
        //Move colar intake in the correct position

        AutoTrajectory goToCoral1 = routine.trajectory("Coral1S2");
        AutoTrajectory goToCoral9 = routine.trajectory("Coral9S1");

        //Go to the coral (coral 1 if station 2, coral 9 if station 1)
        if(targetStation == TargetCoralStation.rightStation){
            goToCoralStation.done().onTrue(IntakeCoralAndGo(goToCoral9));
        }else{
            goToCoralStation.done().onTrue(IntakeCoralAndGo(goToCoral1));
        }
        //Score Coral and come back to the station
        AutoTrajectory goToCoral1R = routine.trajectory("Coral1S2R");
        AutoTrajectory goToCoral9R = routine.trajectory("Coral9S1R");

        goToCoral1.done().onTrue(ScoreCoralAndComeBack(goToCoral1R));     
        goToCoral9.done().onTrue(ScoreCoralAndComeBack(goToCoral9R));

        AutoTrajectory goToCoral2 = routine.trajectory("Coral2S2");
        AutoTrajectory goToCoral10 = routine.trajectory("Coral10S1");

        goToCoral1R.done().onTrue(IntakeCoralAndGo(goToCoral2));
        goToCoral9R.done().onTrue(IntakeCoralAndGo(goToCoral10));




        return routine;
    }
    AutoRoutine getTestRoutine()
    {
        AutoRoutine routine = autoFactory.newRoutine("Test");
        AutoTrajectory testTraj = routine.trajectory("Test");
        AutoTrajectory testTrajReversed = routine.trajectory("TestR");

        routine.active().onTrue(
        Commands.sequence(
            testTraj.resetOdometry(),
            testTraj.cmd()
        )
        );
        //testTraj.done().onTrue(ScoreCoralAndComeBack(testTrajReversed));
        //testTrajReversed.done().onTrue(Commands.runOnce(swerveDriveManager::stopRobot));
        //testTraj.done().onTrue(IntakeCoralAndGo(testTrajReversed));
        return routine;
    }



    private Command ScoreCoralAndComeBack(AutoTrajectory nexTrajectory){
        Command driveToPoseCommand = new AlignToReefCommand(swerveDriveManager, robotContainer.visionManager, null);
        return Commands.sequence(
            Commands.runOnce(swerveDriveManager::stopRobot),
            //driveToPoseCommand.withDeadline(coralAndElevatorManager.getSetElevatorCommand(3)),
            //coralAndElevatorManager.getScoreCoralComand(() -> false), //Should be hasCoral in the future
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory), //Reset PID values for the next trajectory
            nexTrajectory.cmd()
        );
    }
    private Command IntakeCoralAndGo(AutoTrajectory nexTrajectory){
        return Commands.sequence(
                Commands.runOnce(swerveDriveManager::stopRobot),
                //coralAndElevatorManager.getIntakeCoralCommand(() -> false), //Should be hasCoral in the future
                Commands.runOnce(swerveDriveManager::resetAutoTrajectory), //Reset PID values for the next trajectory
                nexTrajectory.cmd()
            );
    }
    private AutoTrajectory getStartingAutoTrajectory(AutoRoutine routine){
        switch (startingPosition) {
            case goonCage:
                if(targetStation == TargetCoralStation.rightStation)
                  return routine.trajectory("AutoStart1S1");
                else
                  return routine.trajectory("AutoStart1S2");
            case sigmaCage:
                if(targetStation == TargetCoralStation.rightStation)
                   return routine.trajectory("AutoStart2S1");
                else
                   return routine.trajectory("AutoStart2S2");
            case edgeCage:
                if(targetStation == TargetCoralStation.rightStation)
                   return routine.trajectory("AutoStart3S1");
                else
                   return routine.trajectory("AutoStart3S2");
            default:
                System.out.println("Wrong auto constants");
                return null;
       }

    }





}
