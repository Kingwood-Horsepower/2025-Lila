package frc.robot.StateMachine;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class PlayerState {
    
    public static CommandXboxController driverController;
    public static PlayerStateMachine player;

    public boolean isExiting = false;

    public PlayerState(){
    }


    public void Enter(){isExiting = false;}

    public void Exit(){isExiting = true;}

    public void onY(){}
    public void onA(){}
    public void onTriggerPressed(){}
    public void onBumperPressed(){}
    public void onTriggerUnpressed(){}
    public void onBumperUnpressed(){}

    public void onX(){}
    public void onB(){}

    public void onPovLeft(){}
    public void onPovRight(){}
    public void onPovDown(){}
    public void onPovUp(){}
    public void onPovCenter(){}

    public void onStart(){}

    public void onBack()
    {
        //onBack Always goes back to drive
        player.coralAndElevatorSubsystem.endIntake();
        player.stateMachine.ChangeState(player.driveState);
    }

}
