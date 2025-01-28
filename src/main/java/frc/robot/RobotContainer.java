// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CameraSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);// Use open-loop control for drive motors
            //.withSteerRequestType(SteerRequestType.); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static CameraSubsystem camera = new CameraSubsystem(); // this was public static final

    public final Trigger targeAquired = new Trigger(() -> camera.isTarget);

    private final double translationVelocityMult = 0.7; //Cannot be more than 1
    private final double rotVelocityMult = 1; 

    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(2, -2, 0); //How fast can the robot accellerate and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(2);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(2);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention, I am so skibidi
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveLimiterX.calculate(driverController.getLeftY()) * translationVelocityMult * MaxSpeed)
                    .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()) * translationVelocityMult * MaxSpeed) 
                    .withRotationalRate(driveLimiterRot.calculate(driverController.getRightX()) * -1 * rotVelocityMult * MaxAngularRate)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverController.b().whileTrue(drivetrain.applyRequest(() ->
            //point.withModuleDirection(new Rotation2d(driverController.getLeftY(), driverController.getLeftX()))
        //));

        targeAquired.and(driverController.b()).whileTrue(camera.runOnce(()-> System.out.println(" target.getYaw()")).andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-1.0 * (camera.targetYaw/50)* MaxAngularRate)) 
        ));// Drive counterclockwise with negative X (left)


    
        driverController.x().whileTrue(Commands.runOnce(() -> drivetrain.resetRotation(null)));    

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

 public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
