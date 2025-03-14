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
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.managers.VisionManager;
import frc.robot.managers.CoralAndElevatorManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Winch;

public class RobotContainer {
    //Data
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final double translationVelocityMult = 0.65; // Cannot be more than 1
    private final double rotVelocityMult = .75;                                                                                      // max angular velocity

    //SwerveRequestes
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)// (not) Use open-loop control for drive motors
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.RobotCentric driveRoboCentric = new SwerveRequest.RobotCentric()
           .withDeadband(MaxSpeed * 0.01)
           .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
           .withDriveRequestType(DriveRequestType.OpenLoopVoltage);// (not) Use open-loop control for drive motors
          //.withSteerRequestType(SteerRequestType.);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionManager visionManager =  new VisionManager(drivetrain);

    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Winch winch = new Winch();
    private final Auto auto;
    private final CoralAndElevatorManager coralAndElevatorManager = new CoralAndElevatorManager();
    
    
    // Other references
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandXboxController driverController = new CommandXboxController(0);
    private final PlayerStateMachine stateMachine = new PlayerStateMachine(driverController);
        
    // SlewRaeLimiters
    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(1.3); // How fast can the robot accellerate                                                                                // and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(1.3);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(2.6);
    private final SlewRateLimiter driveLimiterSlowRot = new SlewRateLimiter(1.1);

    public static final String tagKey = "tag to align to";
    private static int tag = 21;

    // Coral Commands (Some command are public because used by the Auto class)
  //private Command alignRobotWithAprilTag;
    private Command driveToPoseCommand = new DriveToPoseCommand(drivetrain, visionManager, ()->driverController.povRight().getAsBoolean());
    private int inputMult =1;
    private boolean isInRobotCentric = false;

    private Trigger elevatorLimitSwitch = new Trigger(()-> coralAndElevatorManager.getElevator().getIsLimitSwitchZerod());


    public RobotContainer() {     
        configureCommands();
        configureBindings();
        drivetrain.registerTelemetry(logger::telemeterize);
        resetPose();
        auto = new Auto(drivetrain, coralAndElevatorManager, this);

        if (!Preferences.containsKey(tagKey)) {
            Preferences.setDouble(tagKey, tag);
        }
    }

    /* #region configureCommands */
    private void configureCommands() {    
        //align robot with april tag
        //alignRobotWithAprilTag = getAlignWithAprilTagCommand();       
    }
    
    /* #endregion */

    public void loadPreferences() {
        tag = Preferences.getInt(tagKey, tag);
    }

    /* #region configureBindings */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {if (isInRobotCentric){return slowDriveTrainRequest();}
                        else{return drive
                        .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
                                * MaxSpeed)
                        .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
                                * MaxSpeed)
                        .withRotationalRate(driveLimiterRot.calculate(driverController.getRightX()) * -1
                                * rotVelocityMult * MaxAngularRate);}}));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverController.a().onTrue((Commands.runOnce(SignalLogger::start)));
        //driverController.b().onTrue((Commands.runOnce(SignalLogger::stop)));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(driverController.getLeftY(),
        // driverController.getLeftX()))
        // ));

        //driverController.b().whileTrue(alignRobotWithAprilTag);

        // algae intake command
        driverController.leftTrigger(0.1).whileTrue(
            algaeIntake.intake()
        );

        // algae score command
        driverController.leftBumper().whileTrue(
            algaeIntake.score()
        );

        // coral elevator increment level
        driverController.y().onTrue(coralAndElevatorManager.getIncrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        driverController.a().onTrue(coralAndElevatorManager.getDecrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));                
        
        //driverController.b().whileTrue(coralAndElevatorManager.getMoveRollersCommand());

        // coral score command
        // uses stow
        driverController.rightBumper().onTrue(Commands.run(() -> {}));
        driverController.rightBumper().onTrue(coralAndElevatorManager.getScoreCoralComand(() -> !driverController.rightBumper().getAsBoolean()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // coral intake command
        // uses stow
        driverController.start().onTrue(Commands.runOnce(() -> inputMult *= -1));
        
        driverController.b().onTrue(Commands.runOnce(() -> isInRobotCentric = !isInRobotCentric));

        //driverController.b().whileTrue(getAlignWithAprilTagCommand());
        //driverController.x().onTrue(Commands.runOnce(() ->  System.out.println(visionManager.getBestDownTargetOptional().isPresent() ? visionManager.getBestDownTargetOptional().get().getSkew() : -100)));
   
        elevatorLimitSwitch.onTrue(Commands.runOnce(()->coralAndElevatorManager.getElevator().resetEncoders()));

        driverController.x().onTrue(coralAndElevatorManager.getCoralIntake().jiggleIntakeLol());

        driverController.rightTrigger(0.01).onTrue(              
           coralAndElevatorManager.getIntakeCoralCommand(() -> coralAndElevatorManager.hasCoral() | !driverController.rightTrigger().getAsBoolean()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        driverController.povRight().onTrue(driveToPoseCommand);
        driverController.povLeft().onTrue(driveToPoseCommand);
        //---STATE MACHINE ---
        //Only prints stuff right now

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

        driverController.rightBumper().onChange(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onBumper(driverController.rightBumper().getAsBoolean());}));

        driverController.rightTrigger().onChange(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onTrigger(driverController.rightTrigger().getAsBoolean());}));

        driverController.y().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onY();}));

        driverController.a().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onA();}));

        driverController.back().onTrue(Commands.runOnce(
            () -> {stateMachine.getPlayerState().onBack();}));


    }
    /* #endregion */

    /* #region Other Methods*/

    public void resetPose() {
        // The first pose in an autonomous path is often a good choice.
        //var startPose = new Pose2d(new Translation2d(Inches.of(19), Inches.of(44.5)), new Rotation2d(Math.PI));
        var startPose = new Pose2d(AutoConstants.getStartingPosition(), new Rotation2d(Math.PI));
        drivetrain.resetPose(startPose);
    }
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
        drivetrain.stopRobot();
        isInRobotCentric = false;
        inputMult =1;
    }

    // public void teleopInit() {
    //     coralAndElevatorManager.getElevator().resetEncoders();
    // }

    /* #endregion */
    int getInputMult(){
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        if(isBlue){
            return -1 * inputMult;
        }else{
            return 1 * inputMult;
        }
    }


    public Command getAlignWithAprilTagCommand()
    {
        //return new ConditionalCommand(alignDriveTrain(), Commands.runOnce(()->{}), camera::hasTarget);
        //return alignDriveTrain();   
        return null;      
    }
    // private Command alignDriveTrain(){
    //     return  drivetrain.applyRequest(() ->  drive
    // //.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDistanceFromApriltagWhenScoring-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
    //     .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
    //         * MaxSpeed  * 0.2 )
    //     .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
    //         * MaxSpeed * 0.2 )
    //      .withRotationalRate(camera.hasDownTarget() ? -1.0 * (camera.getDownTargetSkew())* 2* MaxAngularRate : 0)).andThen(
    //         Commands.runOnce(() ->System.out.println(camera.hasDownTarget() ? -1.0 * (camera.getDownTargetSkew()/3)* MaxAngularRate : 0))
    //      );
    // }
    private SwerveRequest slowDriveTrainRequest(){
        return  driveRoboCentric
    //.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDistanceFromApriltagWhenScoring-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
        .withVelocityX(-driverController.getLeftY()* translationVelocityMult
            * MaxSpeed  * 0.2 )
        .withVelocityY(-driverController.getLeftX() * translationVelocityMult
            * MaxSpeed * 0.2 )
        .withRotationalRate(driverController.getRightX()* -1
            * rotVelocityMult * MaxAngularRate * 0.4);
    }

    public void sendSwerveData() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            
            builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().RawHeading.getRadians(), null);
            }
            });
    }
}
