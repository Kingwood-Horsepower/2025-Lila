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
public class AlignCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10,2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);

    //private int tagToChase = 17;
    protected Pose3d goal = null;

    protected final SwerveDriveManager swerveDriveManager;
    protected final VisionManager visionManager;


    protected static final ProfiledPIDController xController = new ProfiledPIDController(7, 0, 0.5, X_CONSTRAINTS);
    protected static final ProfiledPIDController yController = new ProfiledPIDController(7, 0, 0.5, Y_CONSTRAINTS);
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(15, 0, .5, THETA_CONSTRAINTS);

    protected static boolean startRotationIsOK = false; 

    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //    .withDriveRequestType(DriveRequestType.Velocity);// (not) Use open-loop control for drive motors
    

    public AlignCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager) {
        
        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        
        xController.setTolerance(0.5);
        yController.setTolerance(0.5);
        thetaController.setTolerance(Units.degreesToRadians(.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);   
        
        addRequirements(swerveDriveManager.getDrivetrain());
    }

    public boolean checkStartRotationIsOK() {
        double tolerance = Units.degreesToRadians(1);
        double current = swerveDriveManager.getRobotPose().getRotation().getRadians();
        double target = goal.toPose2d().getRotation().getRadians();
        return (Math.abs(current-target) < tolerance);
    }

    @Override
    public void initialize() {
        // reset controllers

        xController.reset(swerveDriveManager.getRobotPose().getX());
        yController.reset(swerveDriveManager.getRobotPose().getY());
        thetaController.reset(swerveDriveManager.getRobotPose().getRotation().getRadians());

        goal = initializeGoal();
        
        startRotationIsOK = false;
        startRotationIsOK = checkStartRotationIsOK();

    }

    public void execute() {
        ChassisSpeeds speeds;
        if (true) {//!startRotationIsOK
            speeds = new ChassisSpeeds(
                xController.calculate(swerveDriveManager.getRobotPose().getX(), goal.toPose2d().getX()),
                yController.calculate(swerveDriveManager.getRobotPose().getY(), goal.toPose2d().getY()),
                thetaController.calculate(swerveDriveManager.getRobotPose().getRotation().getRadians(), goal.toPose2d().getRotation().getRadians())
            );
        }
        else {
            speeds = new ChassisSpeeds(
                xController.calculate(swerveDriveManager.getRobotPose().getX(), goal.toPose2d().getX()),
                yController.calculate(swerveDriveManager.getRobotPose().getY(), goal.toPose2d().getY()),
                0.0
                );
        }

        

        swerveDriveManager.setSwerveDriveChassisSpeeds(speeds);
    }

    public boolean getIsNearTranslation(double current, double target) {
        double tolerance = 0.03;
        return Math.abs(current-target) < tolerance;
    }

    public boolean getIsNearRotation(double current, double target) {
        double tolerance = Units.degreesToRadians(0.5);
        return Math.abs(current-target) < tolerance;
    }

    public Pose3d initializeGoal() {
        System.out.println("no goal given, goal is set to current robot pose");
        return new Pose3d(swerveDriveManager.getRobotPose());
    }

  
    @Override
    public boolean isFinished() {
        return getIsNearTranslation(swerveDriveManager.getRobotPose().getX(), goal.toPose2d().getX())
        && getIsNearTranslation(swerveDriveManager.getRobotPose().getY(), goal.toPose2d().getY())
        && getIsNearRotation(swerveDriveManager.getRobotPose().getRotation().getRadians(), goal.toPose2d().getRotation().getRadians());
        //return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
    }
    
    @Override
    public void end(boolean isInterrupted){
        
    }
  }