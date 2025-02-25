package frc.robot.commands;


import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

// public Command getAlignWithAprilTagCommand()
// {
//     return  drivetrain.applyRequest(() ->
//     drive.withVelocityX((camera.getTargetRange() - (Constants.CameraConstants.kDesiredDistanceToAprilTag-Constants.CameraConstants.kRobotToRightCam.getX())) * MaxSpeed*0.12559)
//     .withVelocityY(driverController.getLeftX() * MaxSpeed)
//     .withRotationalRate(-1.0 * (camera.getTargetYaw()/50)* MaxAngularRate)).onlyWhile(targetAquired);

// }

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);

    private static final int TAG_TO_CHASE = 18;
    private static final Pose3d GOAL = 
        new Pose3d(
            new Translation3d(3.05, 3.87, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

    private final CommandSwerveDrivetrain drivetrain;
    private final CameraSubsystem cameraSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, THETA_CONSTRAINTS);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       .withDriveRequestType(DriveRequestType.Velocity);// (not) Use open-loop control for drive motors


    /**
    * Creates a new drive to target command.
    *
    * @param drivetrain The subsystem used by this command.
    */
    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, CameraSubsystem cameraSubsystem, Supplier<Pose2d> poseProvider) {
        this.drivetrain = drivetrain;
        this.cameraSubsystem = cameraSubsystem;
        this.poseProvider = poseProvider;
        
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);   
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        //zero zomething?
    }

    public void execute() {
        drivetrain.setControl(
            drive.withVelocityX(xController.calculate(drivetrain.getRobotPose().getX(), GOAL.toPose2d().getX()))
                 .withVelocityY(yController.calculate(drivetrain.getRobotPose().getY(), GOAL.toPose2d().getY()))
                 .withRotationalRate(thetaController.calculate(drivetrain.getRobotPose().getRotation().getRadians(), GOAL.toPose2d().getRotation().getRadians())));

        
    }
  
    @Override
    public boolean isFinished() {
        return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
    }
  }