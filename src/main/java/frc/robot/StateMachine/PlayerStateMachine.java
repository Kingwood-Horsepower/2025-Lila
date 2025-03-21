package frc.robot.StateMachine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateMachine.states.AlgeePickupState;
import frc.robot.StateMachine.states.AlignmentState;
import frc.robot.StateMachine.states.DriveState;
import frc.robot.StateMachine.states.IntakeState;
import frc.robot.StateMachine.states.ScoringState;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralAndElevatorSubsystem;
import frc.robot.subsystems.Winch;

public class PlayerStateMachine {
    public StateMachine stateMachine;

    //States
    public DriveState driveState;
    public AlignmentState alignmentState;
    public IntakeState intakeState;
    public AlgeePickupState algeePickupState;
    public ScoringState scoringState;

    public SwerveDriveManager swerveDriveManager;
    public VisionManager visionManager;
    public CoralAndElevatorSubsystem  coralAndElevatorSubsystem;
    public Winch winch;
    public AlgaeIntake algaeIntake;

    public boolean estimatedHasCoral = false;

    public PlayerStateMachine(SwerveDriveManager swerveDriveManager, VisionManager visionManager, CoralAndElevatorSubsystem coralAndElevatorSubsystem, Winch winch, AlgaeIntake algaeIntake)
    {
        PlayerState.player = this;
        
        this.swerveDriveManager = swerveDriveManager;
        this.visionManager = visionManager;
        this.coralAndElevatorSubsystem = coralAndElevatorSubsystem;


        driveState = new DriveState();
        alignmentState = new AlignmentState();
        intakeState = new IntakeState();
        algeePickupState = new AlgeePickupState();
        scoringState = new ScoringState();

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
