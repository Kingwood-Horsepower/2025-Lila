package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;


public class AlignToPoseCommand extends AlignCommand {
    private Supplier<Pose2d> goalSupplier;

    public AlignToPoseCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, Supplier<Pose2d> goalSupplier) {
        super(swerveDriveManager, visionManager);
        this.goalSupplier = goalSupplier;
    }

    @Override
    public Pose3d initializeGoal() {
        return new Pose3d(goalSupplier.get());
    }
    
}
