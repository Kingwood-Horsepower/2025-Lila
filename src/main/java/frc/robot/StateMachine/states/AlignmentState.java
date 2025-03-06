package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class AlignmentState extends PlayerState{
    public AlignmentState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        System.out.println("Entered Alignment State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onBumper(){
        player.stateMachine.ChangeState(player.scoringState);
    }

}
