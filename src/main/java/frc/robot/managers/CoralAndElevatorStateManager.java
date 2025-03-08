package frc.robot.managers;

import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.CoralAndElevatorState;

public class CoralAndElevatorStateManager {
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
            Commands.run(() -> coralIntake.setRollerVelocity(-1), coralIntake).until(endCondition),
            moveToNormalState(L4END)
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

    public Command moveToState(CoralAndElevatorState newState) {
        return new ConditionalCommand(
            // if i cant go to this new state
            Commands.none(), 
            // else
            moveToNormalState(newState),
            () -> newState.canComeFrom != null && newState.canComeFrom != lastState.canGoTo
        );
    }

    public Command score(BooleanSupplier endCondition) {
        return Commands.sequence(
            scoringCommand(endCondition),
            moveToState(STOW_DOWN), 
            moveToState(STOW_UP)
        );
    }

    public Command intake(BooleanSupplier endCondition) {
        return Commands.sequence(
            moveToState(INTAKE),
            scoringCommand(endCondition),
            moveToState(STOW_UP)
        );
    }

    public void incrementELevatorScoringLevel() {
        scoringLevel = (scoringLevel + 1) % 5;
        moveToState(scoringStates[scoringLevel]);
    }







    


}
