package frc.robot.StateMachine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateMachine.states.AlgeePickupState;
import frc.robot.StateMachine.states.AlignmentState;
import frc.robot.StateMachine.states.DriveState;
import frc.robot.StateMachine.states.IntakeState;
import frc.robot.StateMachine.states.ScoringState;

public class PlayerStateMachine {
    public StateMachine stateMachine;

    public DriveState driveState = new DriveState();
    public AlignmentState alignmentState = new AlignmentState();
    public IntakeState intakeState = new IntakeState();
    public AlgeePickupState algeePickupState = new AlgeePickupState();
    public ScoringState scoringState = new ScoringState();


    public PlayerStateMachine(CommandXboxController controller)
    {
        PlayerState.driverController = controller;
        PlayerState.player = this;

        stateMachine = new StateMachine(driveState);
    }
    public PlayerState getPlayerState(){
        return stateMachine.getState();
    }
}
