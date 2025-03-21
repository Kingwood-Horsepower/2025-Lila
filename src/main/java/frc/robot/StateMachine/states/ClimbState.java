package frc.robot.StateMachine.states;

import static frc.robot.Constants.AlgaeConstants.ALGAE_DOWN_POINT;

import frc.robot.StateMachine.PlayerState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Winch;

public class ClimbState extends PlayerState{
    //CLIMB STATE SHOULDN'T EXIST, IT SHOULD BE PART OF DRIVE MODE, REASONS:
    //When are you entering this state? Are you really gonna add a button just for that? 
    //When are you exiting this state? What if the driver enters this state accidentaly?

    public ClimbState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.algaeIntake.setSetPoint(ALGAE_DOWN_POINT);
    }
    @Override public void Exit(){
        super.Exit();
        player.algaeIntake.setSetPoint(0.0);
    }

    @Override public void onPovUp(){
        player.winch.winchForwardCommand().schedule();
    }
    @Override public void onPovDown(){
        player.winch.winchReverseCommand().schedule();
    }
    @Override public void onPovCenter(){
        player.winch.winchStopCommand().schedule();
    }

}
