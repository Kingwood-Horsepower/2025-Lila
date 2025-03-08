package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class AlgeePickupState extends PlayerState{
    public AlgeePickupState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        System.out.println("Entered AlgeePickup State");
    }
    @Override public void Exit(){
        super.Exit();
    }

}
