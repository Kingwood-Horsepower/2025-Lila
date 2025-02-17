package frc.robot.commands;


import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 22;
    private static final Transform3d TAG_TO_GOAL = 
        new Transform3d(
            new Translation3d(1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera photonCamera;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, THETA_CONSTRAINTS);

    /**
    * Creates a new drive to target command.
    *
    * @param drivetrain The subsystem used by this command.
    */
    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera photonCamera, Supplier<Pose2d> poseProvider) {
        this.drivetrain = drivetrain;
        this.photonCamera = photonCamera;
        this.poseProvider = poseProvider;
        
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);   
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
    }
  
    @Override
    public boolean isFinished() {
        return true;
    }
  }