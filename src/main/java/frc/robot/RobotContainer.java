// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateMachine.PlayerStateMachine;
import frc.robot.managers.VisionManager;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralAndElevatorSubsystem;
import frc.robot.subsystems.Winch;

public class RobotContainer {
    
    // Subsystems
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Winch winch = new Winch();
    public final Auto auto;
    
    
    // Other references (Managers in public so they can be used in robot)

    private final CommandXboxController driverController = new CommandXboxController(0);
    public final SwerveDriveManager swerveDriveManager = new SwerveDriveManager(driverController);
    public final VisionManager visionManager =  new VisionManager(swerveDriveManager);

    public final CoralAndElevatorSubsystem coralAndElevatorSubsystem = new CoralAndElevatorSubsystem();
    public final PlayerStateMachine stateMachine = new PlayerStateMachine(swerveDriveManager, visionManager, coralAndElevatorSubsystem, winch, algaeIntake);
        
    // Commands and Triggers
    private Trigger elevatorLimitSwitch = new Trigger(()-> coralAndElevatorSubsystem.getElevator().getIsLimitSwitchZerod());

    //these are used for L4 alignment in the coralAndElevatorSubsystem
   
    public RobotContainer() {  
        configureBindings();  
        auto = new Auto(swerveDriveManager, visionManager, coralAndElevatorSubsystem);
    }


    /* #region configureBindings */
    private void configureBindings() {   
        elevatorLimitSwitch.onTrue(Commands.runOnce(()->coralAndElevatorSubsystem.getElevator().resetEncoders()));

        // ---STATE MACHINE ---
        //Bumper and trigger
        driverController.rightBumper().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onBumperPressed();}));

        driverController.rightBumper().onFalse(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onBumperUnpressed();}));

        driverController.rightTrigger().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onTriggerPressed();}));

        driverController.rightTrigger().onFalse(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onTriggerUnpressed();}));

        //Buttons
        driverController.y().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onY();})
        );

        driverController.a().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onA();})
        );

        driverController.b().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onB();})
        );
    
        driverController.x().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onX();})
        );    
        
        //DPAD
        driverController.povLeft().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onPovLeft();})
        );

        driverController.povRight().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onPovRight();})
        );

        driverController.povDown().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onPovDown();})
        );

        driverController.povUp().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onPovUp();})
        );

        driverController.povCenter().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onPovCenter();})
        );
        

        //Other
        driverController.start().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onStart();}
        ));

        driverController.back().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onBack();})
        );




        //ALGEE
        driverController.leftTrigger(0.1).whileTrue(
            algaeIntake.intake()
        );

        driverController.leftBumper().whileTrue(
            algaeIntake.score()
        );

    }
    /* #endregion */

    /* #region Other Methods*/
    public void UpdateRobotPosition(){
        visionManager.UpdateRobotPosition();
        SmartDashboard.putBoolean("Has Target", visionManager.getBestDownTargetOptional().isPresent());
        
    }

    public void autonomousPeriodic(){
        if(!driverController.b().getAsBoolean()){
            auto.PollAutoRoutine();

        }
    }
    public void disabledAuto(){
        auto.KillAutoRoutine();
        swerveDriveManager.stopRobot();
    }

    public void teleopInit(){
        coralAndElevatorSubsystem.bindTriggers();
    }


    /* #endregion */




}
