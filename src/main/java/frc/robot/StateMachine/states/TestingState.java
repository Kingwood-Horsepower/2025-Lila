package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class TestingState extends PlayerState{
    public TestingState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        //player.stateMachine.ChangeState(player.alignmentState);
        // player.swerveDriveManager.setSwerveToNormalDriveCommand().schedule();
        // player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.setSwerveToNormalDriveCommand());
        player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.setSwerveToNormalDriveCommand());
       

        System.out.println("Entered Test State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onY(){
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();

    }
    @Override public void onA(){
        player.coralAndElevatorSubsystem.decrementElevatorScoringLevel();

    }
    @Override public void onBumperPressed() {
        player.coralAndElevatorSubsystem.score().schedule();
    }


}
