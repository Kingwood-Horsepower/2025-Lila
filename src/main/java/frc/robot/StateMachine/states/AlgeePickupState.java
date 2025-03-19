package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class AlgeePickupState extends PlayerState{
    public AlgeePickupState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyScoringLevel();
        System.out.println("Entered AlgeePickup State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onY(){
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyScoringLevel();
    }
    @Override public void onA(){
        if(player.coralAndElevatorSubsystem.decrementDeAlgaeifyScoringLevel())
            player.stateMachine.ChangeState(player.driveState);
    }

}
