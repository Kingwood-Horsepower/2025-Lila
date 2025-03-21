package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateMachine.PlayerState;

public class ScoringState extends PlayerState{
    public ScoringState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        Commands.sequence(
            player.coralAndElevatorSubsystem.score(),
            //player.swerveDriveManager.goBackCommand().withTimeout(0.2),
            Commands.runOnce(() -> player.stateMachine.ChangeState(player.driveState))
        ).schedule();

        player.estimatedHasCoral = false;
        System.out.println("Entered Score State");
    }
    @Override public void Exit(){

        super.Exit();
        player.coralAndElevatorSubsystem.moveDown();
    }



}

