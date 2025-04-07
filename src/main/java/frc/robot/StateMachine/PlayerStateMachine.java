package frc.robot.StateMachine;

import frc.robot.StateMachine.states.AlgeePickupState;
import frc.robot.StateMachine.states.AlignmentState;
import frc.robot.StateMachine.states.DriveState;
import frc.robot.StateMachine.states.IntakeState;
import frc.robot.StateMachine.states.ScoringState;
import frc.robot.StateMachine.states.TestingState;
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
    public TestingState testingState;

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
        this.winch = winch;
        this.algaeIntake = algaeIntake;


        driveState = new DriveState();
        alignmentState = new AlignmentState();
        intakeState = new IntakeState();
        algeePickupState = new AlgeePickupState();
        scoringState = new ScoringState();
        testingState = new TestingState();

        stateMachine = new StateMachine(driveState, testingState);
    }
    public PlayerState getPlayerState(){
        return stateMachine.getState();
    }

    public void startStateMachine()
    {
        if(stateMachine.getState() == alignmentState)
            coralAndElevatorSubsystem.moveDown();
        else {
            coralAndElevatorSubsystem.endIntake();
            coralAndElevatorSubsystem.resetAllIncrements();
        }
            

        
    
        stateMachine.startStateMachine();
    }

    public void startStateMachineTest() 
    {
        if(stateMachine.getState() == alignmentState) coralAndElevatorSubsystem.moveDown();
        stateMachine.startStateMachineTest();

    }
}
