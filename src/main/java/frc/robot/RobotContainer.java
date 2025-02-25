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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CameraSubsystem;

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
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake(elevator);
    private final Auto auto;

    // Other references
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandXboxController driverController = new CommandXboxController(0);
    public final Trigger targetAquired = new Trigger(() -> camera.hasTarget());
        
    // SlewRaeLimiters
    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(1.3); // How fast can the robot accellerate                                                                                // and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(1.3);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(1.3);

    // Coral Commands (Some command are public because used by the Auto class)
    private Command scoreCoral;
    private Command intakeCoral;
    private Command incrementElevatorLevel;
    private Command decrementElevatorLevel;

    private Command alignRobotWithAprilTag;
    private Command driveToPoseCommand = new DriveToPoseCommand(drivetrain, camera, null);


    public RobotContainer() {     
        configureCommands();
        configureBindings();
        drivetrain.registerTelemetry(logger::telemeterize);
        resetPose();
        auto = new Auto(drivetrain, coralIntake, elevator, this);
    }

    /* #region configureCommands */
    private void configureCommands() {
        // Setup Coral commands
        scoreCoral = getScoreCoralComand();

        intakeCoral = getIntakeCoralCommand(() -> coralIntake.hasCoral || !driverController.rightTrigger(0.01).getAsBoolean());

        //increment elevator
        incrementElevatorLevel = Commands.startEnd(
            () -> {
                elevator.incrementElevatorLevel();
                elevator.setElevatorLevel();
                coralIntake.stowIntake();
            },
            () -> {},
            coralIntake, elevator).until(elevator::getIsNearSetPoint);

        //decrement elevator 
        decrementElevatorLevel = Commands.startEnd(
            () -> {
                elevator.decrementElevatorLevel();
                elevator.setElevatorLevel();
                coralIntake.stowIntake();
            },
            () -> {},
            coralIntake, elevator).until(elevator::getIsNearSetPoint);
        
        //align robot with april tag
        alignRobotWithAprilTag = getAlignWithAprilTagCommand();




        
    }
    public Command getAutonomousCommand() { 
        return Commands.print("No autonomous command configured");
        
    }
    
    /* #endregion */

    /* #region configureBindings */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()) * translationVelocityMult
                                * MaxSpeed)
                        .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()) * translationVelocityMult
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

        driverController.b().whileTrue(alignRobotWithAprilTag);

        // algae intake command
        driverController.leftTrigger(0.1).whileTrue(
            algaeIntake.intake()
        );

        // algae score command
        driverController.leftBumper().whileTrue(
            algaeIntake.score()
        );

        // coral elevator increment level
        driverController.y().onTrue(incrementElevatorLevel.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        driverController.a().onTrue(decrementElevatorLevel.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));                

        //uses stow
        driverController.rightBumper().onTrue(Commands.runOnce(() -> {System.out.println("Align with april tag");}));
        driverController.rightBumper().onFalse(scoreCoral.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        //coral intake command
        // uses stow
        driverController.povUp().onTrue(
            driveToPoseCommand.onlyIf(() -> camera.getBestOverallResult().getBestTarget().getFiducialId() == 18));

        driverController.rightTrigger(0.01).onTrue(                
            intakeCoral.onlyWhile(driverController.rightTrigger(0.01):: getAsBoolean).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
                


    }
    /* #endregion */

    /* #region Other Methods*/
    public void UpdateRobotPosition() {
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
    /* #endregion */

    public Command getIntakeCoralCommand(BooleanSupplier conditionForStoppingTheIntake){
        //Intake Coral
        Command runIntake = Commands.startEnd(()->{coralIntake.runIntake(.07, .7);}, ()->{}, coralIntake);
                // intakeCoral = Commands.startEnd(
                //     () -> {
                //         coralIntake.runIntake(.07, .7);
                //         elevator.setElevatorLevel(0);
                //     },
                //     () -> coralIntake.stowIntake(),
                //     coralIntake, elevator).until(()-> elevator.getIsNearSetPoint() && coralIntake.getIsNearSetPoint());
        Command setCor = Commands.startEnd(()->{coralIntake.setSetPoint(0.26);}, ()->{}, coralIntake, elevator );
        Command el =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(0);
            }, () -> {}, coralIntake, elevator);
        return Commands.sequence(
            setCor.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()).unless( elevator :: getIsNearZero),
            el.until(elevator :: getIsNearZero),
            runIntake.until(conditionForStoppingTheIntake),
            Commands.runOnce(coralIntake :: stowIntake, elevator, coralIntake));

    }
    public Command getScoreCoralComand(){
          //Score Coral
          Command setCoralIntakeToLevelCommand = Commands.startEnd(()->{
            if(elevator.getElevatorLevel() == 4) {
                coralIntake.setSetPoint(0.25);
            } else{coralIntake.setSetPoint(0.26);}
         }, ()->{}, coralIntake, elevator );

          Command outTakeCoralCommand = Commands.startEnd(() -> coralIntake.setRollerVelocity(-1), () -> coralIntake.setRollerVelocity(0), elevator, coralIntake);

          Command elevatorToZeroCommand = Commands.startEnd(
              () -> {
                  elevator.setElevatorLevel(0);
              }, () -> {}, coralIntake, elevator);
  
          return Commands.sequence(
              setCoralIntakeToLevelCommand.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()),
              new WaitCommand(.3),
              outTakeCoralCommand.withTimeout(1.5),
              elevatorToZeroCommand.until(() -> elevator.getIsNearZero()),
              Commands.runOnce(coralIntake :: stowIntake, elevator, coralIntake)
          );
    }
    public Command getAlignWithAprilTagCommand()
    {
        return  drivetrain.applyRequest(() ->
        drive.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDistanceFromApriltagWhenScoring-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
        .withVelocityY(driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-1.0 * (camera.getTargetYaw()/50)* MaxAngularRate)).onlyWhile(targetAquired);

    }
}
