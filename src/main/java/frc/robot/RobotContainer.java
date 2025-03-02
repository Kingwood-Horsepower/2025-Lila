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
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.CoralAndElevatorManager;

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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);// (not) Use open-loop control for drive motors
           //.withSteerRequestType(SteerRequestType.);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static CameraSubsystem camera = new CameraSubsystem(); // this was public static final
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Auto auto;
    private final CoralAndElevatorManager coralAndElevatorManager = new CoralAndElevatorManager();
    

    // Other references
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandXboxController driverController = new CommandXboxController(0);
    public final Trigger targetAquired = new Trigger(() -> camera.hasDownTarget());
        
    // SlewRaeLimiters
    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(1.3); // How fast can the robot accellerate                                                                                // and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(1.3);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(2.6);

    // Coral Commands (Some command are public because used by the Auto class)
  private Command alignRobotWithAprilTag;
    private Command driveToPoseCommand = new DriveToPoseCommand(drivetrain, camera, null);
    private int inputMult =1;


    public RobotContainer() {     
        configureCommands();
        configureBindings();
        drivetrain.registerTelemetry(logger::telemeterize);
        resetPose();
        auto = new Auto(drivetrain, coralAndElevatorManager, this);
    }

    /* #region configureCommands */
    private void configureCommands() {    
        //align robot with april tag
        alignRobotWithAprilTag = getAlignWithAprilTagCommand();       
    }
    
    /* #endregion */

    /* #region configureBindings */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
                                * MaxSpeed)
                        .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
                                * MaxSpeed)
                        .withRotationalRate(driveLimiterRot.calculate(driverController.getRightX()) * -1
                                * rotVelocityMult * MaxAngularRate)));

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
        driverController.y().onTrue(coralAndElevatorManager.getIncrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        driverController.a().onTrue(coralAndElevatorManager.getDecrementElevatorCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));                
        
        //driverController.b().whileTrue(coralAndElevatorManager.getMoveRollersCommand());

        // coral score command
        // uses stow
        driverController.rightBumper().onTrue(Commands.run(() -> {}));
        driverController.rightBumper().onTrue(coralAndElevatorManager.getScoreCoralComand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // coral intake command
        // uses stow
        driverController.start().onTrue(Commands.runOnce(() -> inputMult *= -1));
        
        driverController.b().whileTrue(slowDriveTrainCommand());

        //driverController.b().whileTrue(getAlignWithAprilTagCommand());
        driverController.x().onTrue(Commands.runOnce(() ->  System.out.println(camera.hasDownTarget() ? camera.getDownTargetSkew() : -100)));
   
        driverController.rightTrigger(0.01).onTrue(              
           coralAndElevatorManager.getIntakeCoralCommand(() -> coralAndElevatorManager.hasCoral() | !driverController.rightTrigger().getAsBoolean()).onlyWhile(driverController.rightTrigger():: getAsBoolean).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        //driverController.povUp().onTrue(
            //driveToPoseCommand.onlyIf(() -> camera.getBestTarget().getFiducialId() == 18));

    }
    /* #endregion */

    /* #region Other Methods*/
    public void UpdateRobotPosition() {
        //System.out.println("swerveRot: " + drivetrain.getRobotPose().getRotation().getRadians());
        //System.out.println("swervePos: " + drivetrain.getRobotPose().getTranslation());

        if (camera != null) {
            //Right camera
            var visionEst = camera.getEstimatedGlobalRightPose();
            visionEst.ifPresent(
                    est -> {
                        //System.out.println("right: " +est.estimatedPose.getTranslation());
                        SmartDashboard.putString("CameraRightOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraRightOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));

                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
            //Left camera

            visionEst = camera.getEstimatedGlobalLeftPose();
            visionEst.ifPresent(
                    est -> {
                        //System.out.println("left: " + est.estimatedPose.getTranslation());
                        SmartDashboard.putString("CameraLeftOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));
        
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
        }

    }

    public void resetPose() {
        // The first pose in an autonomous path is often a good choice.
        //var startPose = new Pose2d(new Translation2d(Inches.of(19), Inches.of(44.5)), new Rotation2d(Math.PI));
        var startPose = new Pose2d(AutoConstants.getStartingPosition(), new Rotation2d(Math.PI));
        drivetrain.resetPose(startPose);
    }

    public void autonomousPeriodic(){
        if(!driverController.b().getAsBoolean()){
            auto.PollAutoRoutine();

        }
    }
    public void disabledAuto(){
        auto.KillAutoRoutine();
        drivetrain.stopRobot();
    }
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
        return alignDriveTrain();
        
        

       
    }
    private Command alignDriveTrain(){
        return  drivetrain.applyRequest(() ->  drive
    //.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDistanceFromApriltagWhenScoring-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
        .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
            * MaxSpeed  * 0.2 )
        .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
            * MaxSpeed * 0.2 )
         .withRotationalRate(camera.hasDownTarget() ? -1.0 * (camera.getDownTargetSkew())* 2* MaxAngularRate : 0)).andThen(
            Commands.runOnce(() ->System.out.println(camera.hasDownTarget() ? -1.0 * (camera.getDownTargetSkew()/3)* MaxAngularRate : 0))
         );
    }
    private Command slowDriveTrainCommand(){
        return  drivetrain.applyRequest(() ->  drive
    //.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDistanceFromApriltagWhenScoring-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
        .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
            * MaxSpeed  * 0.2 )
        .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
            * MaxSpeed * 0.2 )
            .withRotationalRate(driveLimiterRot.calculate(driverController.getRightX()) * -1
            * rotVelocityMult * MaxAngularRate * 0.2));
    }
}
