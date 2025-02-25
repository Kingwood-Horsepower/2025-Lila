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
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.CoralAndElevatorManager;

import static frc.robot.Constants.AutoConstants.*;

public class Auto {
    private final AutoFactory autoFactory;
    private final CoralAndElevatorManager coralAndElevatorManager;
    private final RobotContainer robotContainer;

    private final AutoRoutine autoRoutine;

    public Auto(CommandSwerveDrivetrain driveSubsystem, CoralAndElevatorManager _CoralAndElevatorManager, RobotContainer _robotContainer)
    {   
        autoFactory = new AutoFactory(
            driveSubsystem::getRobotPose, // A function that returns the current robot pose
            driveSubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            driveSubsystem // The drive subsystem
        );

        coralAndElevatorManager = _CoralAndElevatorManager;

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
        testTraj.done().onTrue(IntakeCoralAndGo(testTrajReversed));
        return routine;
    }



    private Command ScoreCoralAndComeBack(AutoTrajectory nexTrajectory){
        return Commands.sequence(
            robotContainer.getAlignWithAprilTagCommand().withDeadline(coralAndElevatorManager.getSetElevatorCommand(4)),
            coralAndElevatorManager.getScoreCoralComand(),
            nexTrajectory.resetOdometry(),
            nexTrajectory.cmd()
        );
    }
    private Command IntakeCoralAndGo(AutoTrajectory nexTrajectory){
        return Commands.sequence(
                coralAndElevatorManager.getIntakeCoralCommand(() -> coralAndElevatorManager.hasCoral() || robotContainer.driverController.b().getAsBoolean()),
                nexTrajectory.resetOdometry(),
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
