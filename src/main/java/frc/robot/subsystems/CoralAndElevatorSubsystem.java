package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.CoralAndElevatorState;

public class CoralAndElevatorSubsystem {
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake();

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
            Commands.run(() -> coralIntake.setRollerVelocity(-1), coralIntake).until(endCondition)
            //moveToNormalState(L4END)
            //i will add something here
        );
    }

    private Command scoringCommand(BooleanSupplier endCondition) {
        return new ConditionalCommand(
            // if lastState = L4
            rizzTheLevel4GyattCommand(endCondition), 
            // else
            Commands.run(() -> coralIntake.setRollerVelocity(-1), coralIntake).until(endCondition), 

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
                System.out.println(lastState.toString())
;            }), 
            // else
            moveToNormalState(newState),
            () -> newState.canComeFrom != null && newState.canComeFrom != lastState.canGoTo
        );
    }

    public void scheduleNewState(CoralAndElevatorState newState) {
        moveToState(newState).schedule();
    }

    public void score() {
        scoringCommand(() -> coralIntake.runningLowAmps()).schedule();
    }

    public void moveDown() {
        Commands.sequence(
            moveToState(STOW_DOWN), 
            moveToState(STOW_UP)
        ).schedule();
    }

    public void startIntake() {
        moveToState(INTAKE).schedule();
    }

    public void endIntake() {
        moveToState(STOW_UP).schedule();
    }


    public void incrementELevatorScoringLevel() {
        scoringLevel = Math.min(scoringLevel + 1, 4);
        moveToState(scoringStates[scoringLevel]).schedule();
    }

    public void decrementELevatorScoringLevel() {
        scoringLevel = Math.max(scoringLevel - 1, 0);
        moveToState(scoringStates[scoringLevel]).schedule();
    }

    public void incrementDeAlgaeifyScoringLevel() {
        deAlgaeifyLevel = Math.min(deAlgaeifyLevel + 1, 2);
        moveToState(deAlgaeifyStates[deAlgaeifyLevel]).schedule();
    }

    public void decrementDeAlgaeifyScoringLevel() {
        deAlgaeifyLevel = Math.max(deAlgaeifyLevel - 1, 0);
        moveToState(deAlgaeifyStates[deAlgaeifyLevel]).schedule();
    }








    


}
