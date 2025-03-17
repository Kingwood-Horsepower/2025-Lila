package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToProcessorCommand extends DriveToPoseCommand {

    public AlignToProcessorCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, BooleanSupplier isRightStation) {
        if (isRightstation.getAsBoolean()) goal = 
        else goal

        super(swerveDriveManager, visionManager, goal);
    }
    
}
