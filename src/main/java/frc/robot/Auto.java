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
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

import static frc.robot.Constants.AutoConstants.*;

public class Auto {
    private final AutoFactory autoFactory;

    private final CoralIntake coralIntake;
    private final Elevator elevator;

    private Command scoreCoral;
    private Command intakeCoral;

    public Auto(CommandSwerveDrivetrain driveSubsystem, CoralIntake _coralIntake, Elevator _elevator, Command _intakeCoral, Command _scoreCoral)
    {   
        autoFactory = new AutoFactory(
            driveSubsystem::getRobotPose, // A function that returns the current robot pose
            driveSubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            driveSubsystem // The drive subsystem
        );

        coralIntake = _coralIntake;
        elevator = _elevator;

        scoreCoral = _scoreCoral;
        intakeCoral = _intakeCoral;
    }
//1 is closest to center, and S1 is to the right
    @SuppressWarnings("unused")
    public AutoRoutine getAutoRoutine()
    {
        AutoRoutine routine = autoFactory.newRoutine("Autonomous");
        AutoTrajectory goToCoralStation = null;
        
        //custom starting trajectory based on the starting position and the targetStation
        switch (startingPosition) {
            case goonCage:
                if(targetStation == TargetCoralStation.rightStation)
                   goToCoralStation = routine.trajectory("AutoStart1S1");
                else
                   goToCoralStation = routine.trajectory("AutoStart1S2");
                break;
            case sigmaCage:
                if(targetStation == TargetCoralStation.rightStation)
                   goToCoralStation = routine.trajectory("AutoStart2S1");
                else
                   goToCoralStation = routine.trajectory("AutoStart2S2");
                break;
            case edgeCage:
                if(targetStation == TargetCoralStation.rightStation)
                   goToCoralStation = routine.trajectory("AutoStart3S1");
                else
                   goToCoralStation = routine.trajectory("AutoStart3S2");
                break;
            default:
                System.out.println("Wrong auto constants");
                break;
       }
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
        goToCoral1.done().onTrue(ScoreCoralAndComeBack(null));     
        goToCoral9.done().onTrue(ScoreCoralAndComeBack(null));

        AutoTrajectory goToCoral2 = routine.trajectory("Coral2S2");
        AutoTrajectory goToCoral10 = routine.trajectory("Coral10S1");



        return routine;
    }

    private Command ScoreCoralAndComeBack(AutoTrajectory nexTrajectory){
        Command setElevator =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(4);;
                elevator.setElevatorLevel();
                coralIntake.stowIntake();
            },
            () -> {},
            coralIntake, elevator).until(elevator::getIsNearSetPoint);
        return Commands.sequence(
            setElevator,
            scoreCoral,
            nexTrajectory.resetOdometry(),
            nexTrajectory.cmd()
        );
    }
    private Command IntakeCoralAndGo(AutoTrajectory nexTrajectory){
        return Commands.sequence(
                intakeCoral,
                new WaitUntilCommand(coralIntake :: hasCoral),
                nexTrajectory.resetOdometry(),
                nexTrajectory.cmd()
            );
    }





}
