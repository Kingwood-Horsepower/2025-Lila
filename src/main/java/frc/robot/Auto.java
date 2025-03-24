package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlignToL4Constants;
import frc.robot.Constants.AutoConstants.TargetCoralStation;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralAndElevatorSubsystem;

import static frc.robot.Constants.AutoConstants.*;

public class Auto {
    private final AutoFactory autoFactory;
    private final AutoRoutine autoRoutine;

    private final SwerveDriveManager swerveDriveManager;
    private final VisionManager visionManager;
    private final CoralAndElevatorSubsystem coralAndElevatorSubsystem;

    public Auto(SwerveDriveManager swerveDriveManager,  VisionManager visionManager, CoralAndElevatorSubsystem coralAndElevatorSubsystem)
    {   
        autoFactory = new AutoFactory(
            swerveDriveManager::getRobotPose, // A function that returns the current robot pose
            swerveDriveManager::resetPose, // A function that resets the current robot pose to the provided Pose2d
            swerveDriveManager::followPath, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            swerveDriveManager.getDrivetrain() // The drive subsystem
        );

        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        this.coralAndElevatorSubsystem = coralAndElevatorSubsystem;

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
    AutoRoutine getLeftAutoRoutine()
    {
        AutoRoutine leftRoutine = autoFactory.newRoutine("LeftAuto");

        AutoTrajectory goToCoral6 = leftRoutine.trajectory("Start6S2");

        leftRoutine.active().onTrue(Commands.sequence(
            goToCoral6.resetOdometry(),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            Commands.runOnce(coralAndElevatorSubsystem::incrementElevatorScoringLevel),
            goToCoral6.cmd()
        ));

        AutoTrajectory coral6R = leftRoutine.trajectory("Coral6S2R");

        goToCoral6.done().onTrue(ScoreCoralAndComeBack(coral6R, false));


        AutoTrajectory coral4 = leftRoutine.trajectory("Coral4S2");
        coral6R.done().onTrue(IntakeCoralAndGo(coral4));

        AutoTrajectory coral4R = leftRoutine.trajectory("Coral4S2R");
        coral4.done().onTrue(ScoreCoralAndComeBack(coral4R, false));

        AutoTrajectory coral3 = leftRoutine.trajectory("Coral3S2");
        coral4R.done().onTrue(IntakeCoralAndGo(coral3));

        AutoTrajectory coral3R= leftRoutine.trajectory("Coral3S2R");
        coral3.done().onTrue(ScoreCoralAndComeBack(coral3R, false));

        return leftRoutine;
    }



    AutoRoutine getTestRoutine()
    {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        AutoTrajectory testTraj = routine.trajectory("Test");
        AutoTrajectory testTrajReversed = routine.trajectory("TestR");

        routine.active().onTrue(
        Commands.sequence(
            testTraj.resetOdometry(),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            Commands.runOnce(coralAndElevatorSubsystem::incrementElevatorScoringLevel),
            testTraj.cmd()
        )
        );
        testTraj.done().onTrue(ScoreCoralAndComeBack(testTrajReversed, true));
        //testTrajReversed.done().onTrue(Commands.runOnce(swerveDriveManager::stopRobot));
        //testTraj.done().onTrue(IntakeCoralAndGo(testTrajReversed));
        return routine;
    }

    public Command dumboBlueRightAutoRoutine1Command() {
        Transform2d l4Transform = new Transform2d(AlignToL4Constants.ROBOT_TO_L4_DISTANCE, 0, new Rotation2d(0));
        //newGoal = newGoal.plus(l4Transform);
        Pose2d wayPoint1Score = visionManager.getRobotScoringPosition(20, true, true);
        Pose2d wayPoint2Move = new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(220), new Rotation2d(20));//170. 210
        Pose2d wayPoint3Intake = visionManager.getRobotIntakePosition(13);
        Pose2d wayPoint4Score = visionManager.getRobotScoringPosition(19, true, true);
        Pose2d wayPoint5Score = visionManager.getRobotScoringPosition(19, false, true);
        //Command align1stCoral = new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint1Score);


        
        // Command moveAway = new 
        return Commands.sequence(
            //score l4 
            Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint1Score),
            new WaitCommand(0.4),
            coralAndElevatorSubsystem.score(),
            Commands.runOnce(()->coralAndElevatorSubsystem.moveDown()),
            //go to intake
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint2Move),
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint3Intake),
            Commands.runOnce(coralAndElevatorSubsystem::startIntake),
            new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()),
            new WaitCommand(0.2),     
            Commands.runOnce(coralAndElevatorSubsystem::endIntake),
            //score another l4
            Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint4Score),
            new WaitCommand(0.4),
            coralAndElevatorSubsystem.score(),
            Commands.runOnce(()->coralAndElevatorSubsystem.moveDown()),
            //go to intake
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint3Intake),
            Commands.runOnce(coralAndElevatorSubsystem::startIntake),
            new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()),
            new WaitCommand(0.2),     
            Commands.runOnce(coralAndElevatorSubsystem::endIntake),
            //score the third l4???
            Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
            new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint4Score),
            new WaitCommand(0.4),
            coralAndElevatorSubsystem.score(),
            Commands.runOnce(()->coralAndElevatorSubsystem.moveDown())

        );
    }
    



    private Command ScoreCoralAndComeBack(AutoTrajectory nexTrajectory, boolean isRightCoral){
        Pose2d l4ScorePose = visionManager.getRobotScoringPosition(20, true, true);
        Command driveToPoseCommand = new AlignToPoseCommand(swerveDriveManager, visionManager, l4ScorePose);
        Command moveElevatorUpCommand = Commands.runOnce(() -> {
            coralAndElevatorSubsystem.incrementElevatorScoringLevel();
            coralAndElevatorSubsystem.incrementElevatorScoringLevel();
            coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        });



        return Commands.sequence(
            Commands.runOnce(swerveDriveManager::stopRobot),
            moveElevatorUpCommand,
            driveToPoseCommand,
            new WaitCommand(0.4),     
            coralAndElevatorSubsystem.score(),
            Commands.runOnce(() -> coralAndElevatorSubsystem.moveDown()),
            new WaitCommand(0.2),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory), //Reset PID values for the next trajectory
            nexTrajectory.cmd()
        );
    }
    private Command IntakeCoralAndGo(AutoTrajectory nexTrajectory){
        return Commands.sequence(
                Commands.runOnce(swerveDriveManager::stopRobot),
                Commands.runOnce(coralAndElevatorSubsystem::startIntake),
                new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()),
                new WaitCommand(0.2),     
                Commands.runOnce(coralAndElevatorSubsystem::endIntake),
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
            case sigmaCenter:
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
