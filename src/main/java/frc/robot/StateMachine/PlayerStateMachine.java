package frc.robot.StateMachine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateMachine.states.AlgeePickupState;
import frc.robot.StateMachine.states.AlignmentState;
import frc.robot.StateMachine.states.DriveState;
import frc.robot.StateMachine.states.IntakeState;
import frc.robot.StateMachine.states.ScoringState;
import frc.robot.managers.CoralAndElevatorManager;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.CoralAndElevatorSubsystem;

public class PlayerStateMachine {
    public StateMachine stateMachine;

    //States
    public DriveState driveState = new DriveState();
    public AlignmentState alignmentState = new AlignmentState();
    public IntakeState intakeState = new IntakeState();
    public AlgeePickupState algeePickupState = new AlgeePickupState();
    public ScoringState scoringState = new ScoringState();

    public SwerveDriveManager swerveDriveManager;
    public VisionManager visionManager;
    public CoralAndElevatorSubsystem  coralAndElevatorSubsystem;

    public PlayerStateMachine(SwerveDriveManager swerveDriveManager, VisionManager visionManager, CoralAndElevatorSubsystem coralAndElevatorSubsystem)
    {
        PlayerState.player = this;
        
        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        this.coralAndElevatorSubsystem = coralAndElevatorSubsystem;
        stateMachine = new StateMachine(driveState);
    }
    public PlayerState getPlayerState(){
        return stateMachine.getState();
    }

    public void startStateMachine()
    {
        stateMachine.startStateMachine();
    }
}
