package frc.robot.commands;


import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** drive to score command!!! yay. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10,2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);

    private int tagToChase = 17;
    private Pose3d goal = 
        new Pose3d(
            new Translation3d(3.05, 3.87, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

    private final SwerveDriveManager swerveDriveManager;
    private final VisionManager visionManager;

    private BooleanSupplier isRight;

    private final ProfiledPIDController xController = new ProfiledPIDController(7, 0, 0.2, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(7, 0, 0.2, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(10, 0, 0, THETA_CONSTRAINTS);

    // private final PIDController xController = new PIDController(10, 0, 0);
    // private final PIDController yController = new PIDController(10, 0, 0);
    // private final PIDController thetaController = new PIDController(10, 0, 0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       .withDriveRequestType(DriveRequestType.Velocity);// (not) Use open-loop control for drive motors
    
    


    /**
    * Creates a new drive to target command.
    *
    * @param drivetrain The subsystem used by this command.
    */
    public DriveToPoseCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager,BooleanSupplier isRight) {
        
        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        this.isRight = isRight;
        
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);   
        
        addRequirements(swerveDriveManager.getDrivetrain());
    }

    @Override
    public void initialize() {
        // reset controllers

        // reset(double measuredPosition, double measuredVelocity)
        xController.reset(swerveDriveManager.getRobotPose().getX());
        yController.reset(swerveDriveManager.getRobotPose().getY());
        thetaController.reset(swerveDriveManager.getRobotPose().getRotation().getRadians());
        // tagToChase = visionManager.getBestDownTargetOptional().get().getFiducialId();
        // goal = new Pose3d(cameraSubsystem.getCoralScoreTransform(tagToChase, isRight.getAsBoolean()));
        goal = new Pose3d(visionManager.getRobotScoringPosition(isRight.getAsBoolean()));
        System.out.print(" tag id" + tagToChase);
    }

    public void execute() {
        // System.out.print(" xerror " + xController.getPositionError());
        // System.out.print(" yerror " + yController.getPositionError());
        ChassisSpeeds speeds = new ChassisSpeeds(
            -xController.calculate(swerveDriveManager.getRobotPose().getX(), goal.toPose2d().getX()),
            -yController.calculate(swerveDriveManager.getRobotPose().getY(), goal.toPose2d().getY()),
            thetaController.calculate(swerveDriveManager.getRobotPose().getRotation().getRadians(), goal.toPose2d().getRotation().getRadians())
        );

        swerveDriveManager.setSwerveDriveChassisSpeeds(speeds);
    }

    public boolean getIsNear(double current, double target) {
        double tolerance = 0.03;
        return Math.abs(current-target) < tolerance;
    }


  
    @Override
    public boolean isFinished() {
        return getIsNear(swerveDriveManager.getRobotPose().getX(), goal.toPose2d().getX())
        && getIsNear(swerveDriveManager.getRobotPose().getY(), goal.toPose2d().getY())
        && getIsNear(swerveDriveManager.getRobotPose().getRotation().getRadians(), goal.toPose2d().getRotation().getRadians());
        //return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
    }
  }