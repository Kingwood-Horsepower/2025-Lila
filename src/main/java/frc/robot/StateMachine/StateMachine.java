package frc.robot.StateMachine;

import java.lang.Thread.State;

public class StateMachine {

    private PlayerState currentState;

    public StateMachine(PlayerState startingState)
    {
        currentState = startingState;
        currentState.Enter();
    }

    public void ChangeState(PlayerState newState)
    {
        // Don't change state if we are already changing state or we are changing to the same state
        if(currentState.isExiting || newState == currentState) 
            return;
            
        currentState.Exit();
        currentState = newState;
        currentState.Enter();
    }

    public PlayerState getState()
    {
        return currentState;
    }

}
