package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.commands.DriveToPoseCommand;

public class AlignmentState extends PlayerState{
    private Command driveToReefCommand = new DriveToPoseCommand(player.swerveDriveManager, player.visionManager, ()->driverController.b().getAsBoolean());
    private Command driveToClosestReefCommand = new DriveToPoseCommand(player.swerveDriveManager, player.visionManager, null);

    public AlignmentState(){
        super();
    }
    @Override public void Enter()
    {
        super.Enter();
        //driveToClosestReefCommand.schedule();
        //Elevator to 1
        System.out.println("Entered Alignment State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onBumperPressed(){
        player.stateMachine.ChangeState(player.scoringState);
    }

    @Override public void onX(){
        //driveToReefCommand.schedule();
    }
    @Override public void onB(){
        //driveToReefCommand.schedule();
    }


}
