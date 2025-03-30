package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AlignToL4Constants;
import frc.robot.commands.AlignToStationCommand;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.CoralAndElevatorSubsystem;

import static frc.robot.Constants.ElevatorConstants.STOW_UP;
import static frc.robot.Constants.ElevatorConstants.STOW_DOWN;

public class Auto {
    //Auto Classes
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    //References
    private final SwerveDriveManager swerveDriveManager;
    private final VisionManager visionManager;
    private final CoralAndElevatorSubsystem coralAndElevatorSubsystem;

    public Auto(SwerveDriveManager swerveDriveManager,  VisionManager visionManager, CoralAndElevatorSubsystem coralAndElevatorSubsystem)
    {   
        //Create Auto Factory
        autoFactory = new AutoFactory(
            swerveDriveManager::getRobotPose, // A function that returns the current robot pose
            swerveDriveManager::resetPose, // A function that resets the current robot pose to the provided Pose2d
            swerveDriveManager::followPath, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            swerveDriveManager.getDrivetrain() // The drive subsystem
        );

        //Set references
        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        this.coralAndElevatorSubsystem = coralAndElevatorSubsystem;

        //Auto Chooser
        autoChooser = new AutoChooser();

        //autoChooser.addCmd("dumboBlueRightAutoRoutine", ()->m_robotContainer.auto.dumboBlueRightAutoRoutine1Command());
        autoChooser.addRoutine("Test Routine", () -> getTestRoutine());
        autoChooser.addRoutine("Left Routine", () -> getLeftAutoRoutine());
        autoChooser.addRoutine("Right Routine", () -> getRightAutoRoutine());
        autoChooser.addRoutine("Middle Routine", () -> getMiddleAutoRoutine());
    
        //Put the auto chooser on the dashboard
        SmartDashboard.putData("Selected Autonomous Routine", autoChooser);
    
        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    }
     /* #region AUTO ROUTINES*/
    private AutoRoutine getLeftAutoRoutine()
    {
        AutoRoutine leftRoutine = autoFactory.newRoutine("LeftAuto");

        AutoTrajectory goToCoral6 = leftRoutine.trajectory("Start6S2");

        leftRoutine.active().onTrue(Commands.sequence(
            goToCoral6.resetOdometry(),
            coralAndElevatorSubsystem.moveToState(STOW_UP),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            goToCoral6.cmd()
        ));

        AutoTrajectory coral6R = leftRoutine.trajectory("Coral6S2R");

        goToCoral6.done().onTrue(ScoreCoralAndComeBack(coral6R, false));


        AutoTrajectory coral2 = leftRoutine.trajectory("Coral2S2");
        coral6R.done().onTrue(IntakeCoralAndGo(coral6R, coral2));

        AutoTrajectory coral2R = leftRoutine.trajectory("Coral2S2R");
        coral2.done().onTrue(ScoreCoralAndComeBack(coral2R, false));

        AutoTrajectory coral3 = leftRoutine.trajectory("Coral3S2");
        coral2R.done().onTrue(IntakeCoralAndGo(coral2R, coral3));

        AutoTrajectory coral3R= leftRoutine.trajectory("Coral3S2R");
        coral3.done().onTrue(ScoreCoralAndComeBack(coral3R, true));

        return leftRoutine;
    }

    private AutoRoutine getRightAutoRoutine()
    {
        AutoRoutine rightRoutine = autoFactory.newRoutine("RightAuto");
        
        AutoTrajectory goToCoral9 = rightRoutine.trajectory("Start9S1");

        rightRoutine.active().onTrue(Commands.sequence(
            goToCoral9.resetOdometry(),
            coralAndElevatorSubsystem.moveToState(STOW_UP),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            goToCoral9.cmd()
        ));

        AutoTrajectory coral9R = rightRoutine.trajectory("Coral9S1R");

        goToCoral9.done().onTrue(ScoreCoralAndComeBack(coral9R, true));

        AutoTrajectory coral1 = rightRoutine.trajectory("Coral1S1");
        coral9R.done().onTrue(IntakeCoralAndGo(coral9R, coral1));

        AutoTrajectory coral1R = rightRoutine.trajectory("Coral1S1R");
        coral1.done().onTrue(ScoreCoralAndComeBack(coral1R, true));

        AutoTrajectory coral12 = rightRoutine.trajectory("Coral12S1");
        coral1R.done().onTrue(IntakeCoralAndGo(coral1R, coral12));

        AutoTrajectory coral12R = rightRoutine.trajectory("Coral12S1R");
        coral12.done().onTrue(ScoreCoralAndComeBack(coral12R, false));


        return rightRoutine;
    }

    private AutoRoutine getMiddleAutoRoutine()
    {
        AutoRoutine middleRoutine = autoFactory.newRoutine("MiddleAuto");
        AutoTrajectory goToCoral9 = middleRoutine.trajectory("StartMiddle9S1");

        middleRoutine.active().onTrue(Commands.sequence(
            goToCoral9.resetOdometry(),
            coralAndElevatorSubsystem.moveToState(STOW_UP),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            goToCoral9.cmd()
        ));

        AutoTrajectory coral9R = middleRoutine.trajectory("Coral9S1R");

        goToCoral9.done().onTrue(ScoreCoralAndComeBack(coral9R, true));

        AutoTrajectory coral1 = middleRoutine.trajectory("Coral1S1");
        coral9R.done().onTrue(IntakeCoralAndGo(coral9R, coral1));

        AutoTrajectory coral1R = middleRoutine.trajectory("Coral1S1R");
        coral1.done().onTrue(ScoreCoralAndComeBack(coral1R, true));

        AutoTrajectory coral12 = middleRoutine.trajectory("Coral12S1");
        coral1R.done().onTrue(IntakeCoralAndGo(coral1R, coral12));

        AutoTrajectory coral12R = middleRoutine.trajectory("Coral12S1R");
        coral12.done().onTrue(ScoreCoralAndComeBack(coral12R, false));


        return middleRoutine;
    }


    private AutoRoutine getTestRoutine()
    {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        AutoTrajectory testTraj = routine.trajectory("Test");
        AutoTrajectory testTrajReversed = routine.trajectory("TestR");

        routine.active().onTrue(
        Commands.sequence(
            testTraj.resetOdometry(),
            coralAndElevatorSubsystem.moveToState(STOW_UP),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
            testTraj.cmd()
        )
        );
        testTraj.done().onTrue(ScoreCoralAndComeBack(testTrajReversed, true));
       // testTrajReversed.done().onTrue(Commands.runOnce(swerveDriveManager::stopRobot));


        AutoTrajectory goToCoral2 = routine.trajectory("Coral2S2");
        testTrajReversed.done().onTrue(IntakeCoralAndGo(testTrajReversed, goToCoral2));
        
        AutoTrajectory goToCoral2R = routine.trajectory("Coral2S2R");
        goToCoral2.done().onTrue(ScoreCoralAndComeBack(goToCoral2R, false));
        return routine;
    }
    /* #endregion */

    /* #region COMMAND SEQUENCES*/

    /**
     * Intake 
     * @param nexTrajectory the trajectory to load after scoring
     * @param isRightCoral
     * @return
     */
    

    private Command ScoreCoralAndComeBack(AutoTrajectory nexTrajectory, boolean isRightCoral){
        Command alignToReefCommand = new AlignToPoseCommand(swerveDriveManager, visionManager,
             () -> visionManager.getRobotScoringPosition(isRightCoral));

        //Command alignToPoseCommand = new AlignToPoseCommand(swerveDriveManager, visionManager, () -> nexTrajectory.getInitialPose().get());

        //This is where coding dies
        //Lila please don't watch
        Command iWannaCry =  Commands.sequence(
            new PrintCommand("What is even going on"),
            Commands.runOnce(swerveDriveManager::resetAutoTrajectory), //Reset PID values for the next trajectory (MANDATORY BEFORE EVERY TRAJECTORY)
            nexTrajectory.cmd()
        );
        

        return Commands.sequence(
            Commands.runOnce(swerveDriveManager::stopRobot),
            coralAndElevatorSubsystem.moveToState(STOW_DOWN),
            coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
            coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
            //coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
            //coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
            alignToReefCommand,
            Commands.runOnce(swerveDriveManager::stopRobot),
            new WaitCommand(0.04),
            new WaitUntilCommand(()->coralAndElevatorSubsystem.getIsNearState()).withTimeout(2),
            new WaitCommand(0.1),
            new PrintCommand("Aligned"),
            coralAndElevatorSubsystem.score(),
            new WaitCommand(0.2),
            new PrintCommand("Scored"),
            coralAndElevatorSubsystem.moveDownAutonomousCommand(), 
            new PrintCommand("Moved Down"),
            //THIS COMMAND WILL ---NUKE--- THE SEQUENCE AND START THE NEXT OTHER SEQUENCE
            Commands.runOnce(() -> coralAndElevatorSubsystem.moveToState(STOW_UP)
                                    .andThen(iWannaCry).schedule()) //Kill myself
        );
    }

    public Command scoreTestElevatorCommand(){
        //COMMAND USED FOR TESTING - NOT IN THE ACTUAL ROUTINE

        autoFactory.trajectoryCmd("PracticeField");
        Command alignToReefCommand = new AlignToPoseCommand(swerveDriveManager, visionManager,
        () -> visionManager.getRobotScoringPosition(true));

        //Command alignToPoseCommand = new AlignToPoseCommand(swerveDriveManager, visionManager, () -> nexTrajectory.getInitialPose().get());

        //This is where coding dies
        //Lila please don't watch
        Command iWannaCry =  Commands.sequence(
        new PrintCommand("Test successfull"),
        Commands.runOnce(swerveDriveManager::resetAutoTrajectory) //Reset PID values for the next trajectory (MANDATORY BEFORE EVERY TRAJECTORY)
         //nexTrajectory.cmd()
        );
   

   return Commands.sequence(
       autoFactory.resetOdometry("Coral3S2R"),
       coralAndElevatorSubsystem.moveToState(STOW_UP),
       Commands.runOnce(swerveDriveManager::resetAutoTrajectory),
       //Trajectory
       Commands.runOnce(swerveDriveManager::stopRobot),
       coralAndElevatorSubsystem.moveToState(STOW_DOWN),
       coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
       coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
       //coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
       //coralAndElevatorSubsystem.incrementElevatorScoringLevelCommand(),
       alignToReefCommand,
       Commands.runOnce(swerveDriveManager::stopRobot),
       new WaitCommand(0.04),
       new WaitUntilCommand(()->coralAndElevatorSubsystem.getIsNearState()).withTimeout(2),
       new WaitCommand(0.1),
       new PrintCommand("Aligned"),
       coralAndElevatorSubsystem.score(),
       new WaitCommand(0.2),
       new PrintCommand("Scored"),
       coralAndElevatorSubsystem.moveDownAutonomousCommand(), 
       new PrintCommand("Moved Down"),
       //THIS COMMAND WILL ---NUKE--- THE SEQUENCE AND START THE NEXT OTHER SEQUENCE
       Commands.runOnce(() -> coralAndElevatorSubsystem.moveToState(STOW_UP)
                               .andThen(iWannaCry).schedule()) //Kill myself
   );

    }

    /**
     * Intake Coral, then load and start the next traject
     * @param nexTrajectory the trajectory to load after intaking
     * @return
     */
    private Command IntakeCoralAndGo(AutoTrajectory previousTrajectory, AutoTrajectory nexTrajectory){ 
        Command alignToStationCommand = new AlignToStationCommand(swerveDriveManager, visionManager);


        return Commands.sequence(
                Commands.runOnce(swerveDriveManager::stopRobot),
                coralAndElevatorSubsystem.startIntakeCommand(),
                alignToStationCommand,
                new PrintCommand("Intaking"),
                Commands.runOnce(swerveDriveManager::stopRobot),
                new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()).withTimeout(2),
                new WaitCommand(0.2),  
                new PrintCommand("Finished intaking"),   
                coralAndElevatorSubsystem.endIntakeCommand(),
                new PrintCommand("Ended Intake"),   
                Commands.runOnce(swerveDriveManager::resetAutoTrajectory), //Reset PID values for the next trajectory
                nexTrajectory.cmd()
            );
    }

   /* #endregion */

    /* #region ALBERTS TRASHCAN*/

    // private Command dumboBlueRightAutoRoutine1Command() {
    //     Transform2d l4Transform = new Transform2d(AlignToL4Constants.ROBOT_TO_L4_DISTANCE, 0, new Rotation2d(0));
    //     //newGoal = newGoal.plus(l4Transform);
    //     Pose2d wayPoint1Score = visionManager.getRobotScoringPosition(20, true, true);
    //     Pose2d wayPoint2Move = new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(220), new Rotation2d(20));//170. 210
    //     Pose2d wayPoint3Intake = visionManager.getRobotIntakePosition(13);
    //     Pose2d wayPoint4Score = visionManager.getRobotScoringPosition(19, true, true);
    //     Pose2d wayPoint5Score = visionManager.getRobotScoringPosition(19, false, true);
    //     //Command align1stCoral = new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint1Score);

    //     //I HAD TO COMMMENT THIS CAUSE WE CHANGE THE ALIGN TO POSE COMMAND
        
    //     // Command moveAway = new 
    //     return Commands.sequence(
    //         //score l4 
    //         Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
    //         //new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint1Score),
    //         new WaitCommand(0.4),
    //         coralAndElevatorSubsystem.score(),
    //         Commands.runOnce(()->coralAndElevatorSubsystem.moveDown())
    //         //go to intake
    //         // new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint2Move),
    //         // new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint3Intake),
    //         // Commands.runOnce(coralAndElevatorSubsystem::startIntake),
    //         // new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()),
    //         // new WaitCommand(0.2),     
    //         // Commands.runOnce(coralAndElevatorSubsystem::endIntake),
    //         // //score another l4
    //         // Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
    //         // new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint4Score),
    //         // new WaitCommand(0.4),
    //         // coralAndElevatorSubsystem.score(),
    //         // Commands.runOnce(()->coralAndElevatorSubsystem.moveDown()),
    //         // //go to intake
    //         // new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint3Intake),
    //         // Commands.runOnce(coralAndElevatorSubsystem::startIntake),
    //         // new WaitUntilCommand(()-> coralAndElevatorSubsystem.hasCoral()),
    //         // new WaitCommand(0.2),     
    //         // Commands.runOnce(coralAndElevatorSubsystem::endIntake),
    //         // //score the third l4???
    //         // Commands.runOnce(()->coralAndElevatorSubsystem.moveToElevatorScoringLevel(4)),
    //         // new AlignToPoseCommand(swerveDriveManager, visionManager, wayPoint4Score),
    //         // new WaitCommand(0.4),
    //         // coralAndElevatorSubsystem.score(),
    //         // Commands.runOnce(()->coralAndElevatorSubsystem.moveDown())

    //     );
    // }

   /* #endregion */
}
