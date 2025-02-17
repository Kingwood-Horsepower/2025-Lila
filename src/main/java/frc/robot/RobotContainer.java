// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralIntakeSetAndWaitCommand;
import frc.robot.commands.ElevatorSetAndWaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CameraSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.Velocity);// (not) Use open-loop control for drive motors
    // .withSteerRequestType(SteerRequestType.);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static CameraSubsystem camera = new CameraSubsystem(); // this was public static final

    public final Trigger targeAquired = new Trigger(() -> camera.hasTarget);
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake(elevator);

    private final double translationVelocityMult = 0.15; // Cannot be more than 1
    private final double rotVelocityMult = .5;

    // SlewRaeLimiters
    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(2, -2, 0); // How fast can the robot accellerate
                                                                                 // and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(2);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(2);

    // Algee Commands
    private Command setCoralIntakeToLevelCommand;
    private Command outTakeCoralCommand;
    private Command waitSmol;
    private Command waitLol;
    private Command setRollerVelocitiesZeroCommand;
    private Command elevatorToZeroCommand;
    private Command stowIntakeCommand;

    public RobotContainer() {
        configureCommands();
        configureBindings();
        resetPose();
    }

    /* #region configureCommands */
    private void configureCommands() {
        // Setup Coral and Alge Commands
        setCoralIntakeToLevelCommand = new CoralIntakeSetAndWaitCommand(coralIntake, elevator);
        outTakeCoralCommand = Commands.runOnce(
                () -> coralIntake.setRollerVelocity(-1),
                elevator, coralIntake);
        waitSmol = new WaitCommand(.3);
        waitLol = new WaitCommand(1.5);

        setRollerVelocitiesZeroCommand = Commands.runOnce(
                () -> coralIntake.setRollerVelocity(0),
                elevator, coralIntake);

        elevatorToZeroCommand = new ElevatorSetAndWaitCommand(elevator);

        stowIntakeCommand = Commands.runOnce(
                () -> {

                    coralIntake.stowIntake();
                },
                elevator, coralIntake);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    /* #endregion */

    /* #region configureBindings */
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention, I am so
        // skibidi
        // and Y is defined as to the left according to WPILib convention.
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
        driverController.a().onTrue((Commands.runOnce(SignalLogger::start)));
        driverController.b().onTrue((Commands.runOnce(SignalLogger::stop)));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(driverController.getLeftY(),
        // driverController.getLeftX()))
        // ));

        // targeAquired.and(driverController.b()).whileTrue(camera.runOnce(()->
        // System.out.println(" target.getYaw()")).andThen(drivetrain.applyRequest(() ->
        // drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward
        // with negative Y (forward)
        // .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with
        // negative X (left)
        // .withRotationalRate(-1.0 * (camera.targetYaw/50)* MaxAngularRate))
        // ));// Drive counterclockwise with negative X (left)

        // coral intake command
        // uses stow
        driverController.y().whileTrue(
                Commands.startEnd(
                        () -> {
                            coralIntake.runIntake(.07, .7);
                            elevator.setElevatorLevel(0);
                        },
                        () -> coralIntake.stowIntake(),
                        coralIntake, elevator));

        // algae intake command
        driverController.leftTrigger(0.1).whileTrue(
                Commands.startEnd(
                        () -> algaeIntake.runIntake(.11, -1.0),
                        () -> algaeIntake.runIntake(0.04, 0.0),
                        algaeIntake));

        // algae score command
        driverController.leftBumper().whileTrue(
                Commands.startEnd(
                        () -> algaeIntake.runIntake(.11, 1.0),
                        () -> algaeIntake.runIntake(0.0, 0.0),
                        algaeIntake));

        // coral elevator increment level
        driverController.rightBumper().whileTrue(
                Commands.runOnce(
                        () -> {
                            elevator.incrementElevatorLevel();
                            elevator.setElevatorLevel();
                            coralIntake.stowIntake();
                        },
                        coralIntake, elevator));

        // uses stow
        driverController.rightTrigger(0.01).onTrue(
                setCoralIntakeToLevelCommand.andThen(
                        waitSmol.andThen(
                                outTakeCoralCommand.andThen(
                                        waitLol.andThen(
                                                setRollerVelocitiesZeroCommand.andThen(
                                                        elevatorToZeroCommand.andThen(
                                                                stowIntakeCommand)))))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    /* #endregion */

    /* #region Other Methods*/
    public void UpdateRobotPosition() {
        if (camera != null) {
            var visionEst = camera.getEstimatedGlobalPose();
            visionEst.ifPresent(
                    est -> {
                        System.out.println(est.estimatedPose.getTranslation());
                        // SmartDashboard.putString("string", state.Pose.getTranslation().toString());

                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
        }

    }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.
        var startPose = new Pose2d(new Translation2d(Inches.of(19), Inches.of(44.5)), new Rotation2d());
        drivetrain.resetPose(startPose);
    }
    /* #endregion */
}
