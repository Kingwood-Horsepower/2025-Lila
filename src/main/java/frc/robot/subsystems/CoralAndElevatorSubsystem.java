package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CoralAndElevatorState;


public class CoralAndElevatorSubsystem extends SubsystemBase {
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake();
    //private final SwerveDriveManager swerveDriveManager;
    
    private CoralAndElevatorState lastState;
    private int scoringLevel = 0;

    private final CoralAndElevatorState[] scoringStates = {
        STOW_DOWN, // this is wrong
        L1,
        L2,
        L3,
        L4
    };

    private int deAlgaeifyLevel = 0;

    private final CoralAndElevatorState[] deAlgaeifyStates = {
        STOW_UP,
        L2ALGAE,
        L3ALGAE
    };

    public CoralAndElevatorSubsystem(){
        //lastState = STOW_UP;
        onL4().onTrue(coralIntake.primeCoralForL4());
        onL4().onFalse(coralIntake.retractCoralFromL4());
    }

    // private Command moveToStateTest(CoralAndElevatorState newState) {
    //     return new FunctionalCommand(
    //     () -> {
    //         coralIntake.setSetPoint(deAlgaeifyLevel);
    //     }
    //         , null, null, null, null)
    // } 
    



    private Command moveToNormalState(CoralAndElevatorState newState) {
        return Commands.sequence(
            coralIntake.moveToSetPoint(newState.coralPrePosition),
            elevator.moveToSetPoint(newState.elevatorPosition),
            coralIntake.moveToSetPoint(newState.coralEndPosition),
            Commands.runOnce(() -> coralIntake.setRollerVelocity(newState.runRollers), coralIntake),
            Commands.runOnce(() -> lastState = newState)
            );
    }

    private Command rizzTheLevel4GyattCommand(BooleanSupplier endCondition) {
        return Commands.sequence(
            Commands.run(() -> coralIntake.setRollerVelocity(-.5), coralIntake).until(endCondition)
            .andThen(new WaitCommand(0.4))
            
            //moveToNormalState(L4END)
            //i will add something here
        );
    }

    private Command scoringCommand(BooleanSupplier endCondition) {
        return new ConditionalCommand(
            // if lastState = L4
            rizzTheLevel4GyattCommand(endCondition), 
            // else
            Commands.run(() ->{
             if(lastState == L1)
                coralIntake.setRollerVelocity(-0.5);
             else
                coralIntake.setRollerVelocity(-1);
        
        }, coralIntake).until(endCondition)
            .andThen( new WaitCommand(0.4))
            , 

            () -> lastState == L4);
    }

    // the public commands

    private Command moveToState(CoralAndElevatorState newState) {
        return new ConditionalCommand(
            // if i cant go to this new state
            Commands.runOnce(() -> {
                System.out.print("Error, can't go to this state, (alberts state machine): ");
                System.out.print(newState.toString());
                System.out.print(" old state: ");
                //System.out.println(lastState.toString())
;            }), 
            // else
            moveToNormalState(newState),
            () -> newState.canComeFrom != null && newState.canComeFrom != lastState.canGoTo
        );
    }

    public void scheduleNewState(CoralAndElevatorState newState) {
        moveToState(newState).schedule();
    }

    public Command score() {
        return scoringCommand(() -> coralIntake.runningLowAmps());
    }

    public void moveDown() {
        Commands.sequence(
            moveToState(STOW_DOWN), 
            moveToState(STOW_UP),
            Commands.runOnce(() -> resetAllIncrements())
        ).schedule();
    }

    public Command moveDownCommand() {
        return Commands.sequence(
            moveToState(STOW_DOWN), 
            moveToState(STOW_UP),
            Commands.runOnce(() -> resetAllIncrements())
        );
    }

    public void startIntake() {
        moveToState(INTAKE).schedule();
    }

    public void endIntake() {
        moveToState(STOW_UP).schedule();
    }


    public void incrementElevatorScoringLevel() {
        scoringLevel = Math.min(scoringLevel + 1, 4);
        System.out.println(scoringLevel);
        moveToState(scoringStates[scoringLevel]).schedule();
    }

    public Command incrementElevatorScoringLevelCommand() {
        scoringLevel = Math.min(scoringLevel + 1, 4);
        System.out.println(scoringLevel);
        return moveToState(scoringStates[scoringLevel]);
    }

    public void decrementElevatorScoringLevel() {
        scoringLevel = Math.max(scoringLevel - 1, 0);
        System.out.println(scoringLevel);
        moveToState(scoringStates[scoringLevel]).schedule();
    }

    public void incrementDeAlgaeifyScoringLevel() {
        deAlgaeifyLevel = Math.min(deAlgaeifyLevel + 1, 2);
        moveToState(deAlgaeifyStates[deAlgaeifyLevel]).schedule();
    }

    public boolean decrementDeAlgaeifyScoringLevel() {
        deAlgaeifyLevel = Math.max(deAlgaeifyLevel - 1, 0);
        moveToState(deAlgaeifyStates[deAlgaeifyLevel]).schedule();
        return deAlgaeifyLevel == 0; //It's useful for the player state Machine
    }

    public void moveToElevatorScoringLevel(int scoringLevel) {
        System.out.println(scoringLevel);
        this.scoringLevel = scoringLevel;
        moveToState(scoringStates[scoringLevel]).schedule();
    }

    public void moveToDeAlgaeifyLevel(int deAlgaeifyLevelLevel) {
        System.out.println(deAlgaeifyLevel);
        this.deAlgaeifyLevel = deAlgaeifyLevel;
        moveToState(deAlgaeifyStates[deAlgaeifyLevel]).schedule();
    }

    public void resetAllIncrements() {
        deAlgaeifyLevel = 0;
        scoringLevel = 0;
    } 

    public boolean hasCoral(){
        return coralIntake.hasCoral;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public CoralIntake getCoralIntake() {
        return coralIntake;
    }

    public void jiggleIntake(){
        coralIntake.jiggleIntakeLol( ()-> coralIntake.getRollerEncoderPosition());
    }

    public int getScoringLevel() {
        return scoringLevel;
    }

    public Trigger onL4() {
        return new Trigger(() -> scoringLevel == 4);
    }









    


}
