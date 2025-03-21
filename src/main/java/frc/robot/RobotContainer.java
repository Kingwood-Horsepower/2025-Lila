// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;
import java.util.function.BooleanSupplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.StateMachine.PlayerStateMachine;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.AlignToStationCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.managers.VisionManager;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralAndElevatorSubsystem;
import frc.robot.subsystems.Winch;

public class RobotContainer {
    
    // Subsystems
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Winch winch = new Winch();
    private final Auto auto;
    
    
    // Other references (Managers in public so they can be used in robot)

    private final CommandXboxController driverController = new CommandXboxController(0);
    public final SwerveDriveManager swerveDriveManager = new SwerveDriveManager(driverController);
    public final VisionManager visionManager =  new VisionManager(swerveDriveManager);

    public final CoralAndElevatorSubsystem coralAndElevatorSubsystem = new CoralAndElevatorSubsystem();
    public final PlayerStateMachine stateMachine = new PlayerStateMachine(swerveDriveManager, visionManager, coralAndElevatorSubsystem, winch, algaeIntake);
        

    // public static final String tagKey = "tag to align to";
    // private static int tag = 21;

    // Commands and Triggers
    private Command alignToReefCommand = new AlignToReefCommand(swerveDriveManager, visionManager, ()->driverController.povRight().getAsBoolean(), () -> coralAndElevatorSubsystem.getScoringLevel() == 4);
    private Command alignToStationCommand = new AlignToStationCommand(swerveDriveManager, visionManager);
    private Trigger elevatorLimitSwitch = new Trigger(()-> coralAndElevatorSubsystem.getElevator().getIsLimitSwitchZerod());

    //these are used for L4 alignment in the coralAndElevatorSubsystem
   
    public RobotContainer() {     
        configureCommands();
        configureBindings();  
        auto = new Auto(swerveDriveManager, visionManager, coralAndElevatorSubsystem);

    }

    /* #region configureCommands */
    private void configureCommands() {    
        //align robot with april tag
        //alignRobotWithAprilTag = getAlignWithAprilTagCommand();       
    }
    
    /* #endregion */


    /* #region configureBindings */
    private void configureBindings() {
        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverController.a().onTrue((Commands.runOnce(SignalLogger::start)));
        //driverController.b().onTrue((Commands.runOnce(SignalLogger::stop)));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(driverController.getLeftY(),
        // driverController.getLeftX()))
        // ));

        //driverController.b().whileTrue(alignRobotWithAprilTag);

        // // algae intake command
        // // coral elevator increment level
        // driverController.y().onTrue(coralAndElevatorManager.getIncrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // driverController.a().onTrue(coralAndElevatorManager.getDecrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));                
        
        // //driverController.b().whileTrue(coralAndElevatorManager.getMoveRollersCommand());

        // // coral score command
        // // uses stow
        // driverController.rightBumper().onTrue(Commands.run(() -> {}));
        // driverController.rightBumper().onTrue(coralAndElevatorManager.getScoreCoralComand(() -> !driverController.rightBumper().getAsBoolean()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // coral intake command
        // uses stow
        // driverController.start().onTrue(Commands.runOnce(swerveDriveManager::invertControls));  
        // driverController.b().toggleOnFalse(swerveDriveManager.setSwerveToSlowDriveCommand());

        // //driverController.b().whileTrue(getAlignWithAprilTagCommand());
        // //driverController.x().onTrue(Commands.runOnce(() ->  System.out.println(visionManager.getBestDownTargetOptional().isPresent() ? visionManager.getBestDownTargetOptional().get().getSkew() : -100)));
   
        elevatorLimitSwitch.onTrue(Commands.runOnce(()->coralAndElevatorSubsystem.getElevator().resetEncoders()));

        // driverController.x().onTrue(coralAndElevatorManager.getCoralIntake().jiggleIntakeLol(()->coralAndElevatorManager.getCoralIntake().getRollerEncoderPosition()));

        // driverController.rightTrigger(0.01).onTrue(              
        //    coralAndElevatorManager.getIntakeCoralCommand(() -> coralAndElevatorManager.hasCoral() | !driverController.rightTrigger().getAsBoolean()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        // driverController.povRight().onTrue(alignToReefCommand);
        // driverController.povLeft().onTrue(alignToReefCommand);
        // driverController.rightTrigger().and(driverController.povUp()).onTrue(alignToStationCommand);

        // driverController.povUp().onTrue(Commands.startEnd(
        //     ()-> {
        //         algaeIntake.setSetPoint(AlgaeConstants.ALGAE_DOWN_POINT);
        //         winch.runWinch(-1);
        //     },
        //     ()->winch.runWinch(0), 
        //     winch, algaeIntake));

        //             driverController.povDown().onTrue(Commands.startEnd(
        //             ()-> {
        //                 algaeIntake.setSetPoint(AlgaeConstants.ALGAE_DOWN_POINT);
        //                 winch.runWinch(1);
        //             },
        //             ()->winch.runWinch(0), 
        //             winch, algaeIntake));

        //             driverController.povCenter().onTrue(Commands.startEnd(
        //             ()-> {
        //                 algaeIntake.setSetPoint(AlgaeConstants.ALGAE_DOWN_POINT);
        //                 winch.runWinch(0);
        //             },
        //             ()->winch.runWinch(0), 
        //             winch, algaeIntake));


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

    /* #endregion */




}
