package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class ScoringState extends PlayerState{
    public ScoringState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        System.out.println("Entered Score State");
    }
    @Override public void Exit(){
        super.Exit();
    }

}

