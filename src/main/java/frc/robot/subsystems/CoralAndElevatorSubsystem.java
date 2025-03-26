package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CoralAndElevatorState;


public class CoralAndElevatorSubsystem extends SubsystemBase {
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake();
    //private final SwerveDriveManager swerveDriveManager;
    
    private CoralAndElevatorState lastState = STOW_UP;
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

    private double testSetPoint = 0.0;
    
 
    public void bindTriggers(){
        onL4().onTrue(coralIntake.primeCoralForL4());
        onL4().onFalse(coralIntake.retractCoralFromL4());
    }

    public CoralAndElevatorSubsystem(){
        //lastState = STOW_UP;

        SmartDashboard.putData("IR Override", overrideHasCoral());
        SmartDashboard.putData("Increment", Commands.sequence(incrementElevatorScoringLevelCommand()));
        SmartDashboard.putData("Decrement", Commands.sequence(decrementElevatorScoringLevelCommand()));
        SmartDashboard.putData("testPrint", new PrintCommand("testPrint"));

    }

    private Command moveToNormalState(CoralAndElevatorState newState) {
        Command seq = Commands.sequence(
            coralIntake.moveToSetPoint(newState.coralPrePosition),
            elevator.moveToSetPoint(newState.elevatorPosition),
            coralIntake.moveToSetPoint(newState.coralEndPosition),
            Commands.runOnce(() -> coralIntake.setRollerVelocity(newState.runRollers), coralIntake),
            Commands.runOnce(() -> lastState = newState)
        );
        // adding the full subsystem as a requirement
        seq.addRequirements(this);
        return seq;
    }

    public Command primeCoralForL4() {
        return coralIntake.primeCoralForL4();
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
        Command seq = new ConditionalCommand(
            // if lastState = L4
            rizzTheLevel4GyattCommand(endCondition), 
            // else
            Commands.run(() ->{
                if(lastState == L1)
                    coralIntake.setRollerVelocity(-0.5);
                else
                    coralIntake.setRollerVelocity(-1);
            }, 
            coralIntake).until(endCondition).andThen( new WaitCommand(0.4)), 
            () -> lastState == L4);
        seq.addRequirements(this);
        return seq;
    }

    // the public commands

    public Command moveToState(CoralAndElevatorState newState) {
        return moveToNormalState(newState);
//         return new ConditionalCommand(
//             // if i cant go to this new state
//             Commands.runOnce(() -> {
//                 System.out.print("Error, can't go to this state, (alberts state machine): ");
//                 System.out.print(newState.toString());
//                 System.out.print(" old state: ");
//                 //System.out.println(lastState.toString())
// ;            }), 
//             // else
//             moveToNormalState(newState),
//             () -> newState.canComeFrom != null && newState.canComeFrom != lastState.canGoTo
        //);
    }

    private Command moveToSupplierState(Supplier<CoralAndElevatorState> stateSupplier) {
        return Commands.sequence(
            new PrintCommand("moveToSupplierState has state with elevator distance: " + Double.toString(stateSupplier.get().elevatorPosition)),
            moveToNormalState(stateSupplier.get())
        );
    }

    private CoralAndElevatorState getScoringState(IntSupplier levelSupplier) {
        return scoringStates[levelSupplier.getAsInt()];
    }

    public void scheduleNewState(CoralAndElevatorState newState) {
        moveToState(newState).schedule();
    }

    public Command score() {
        return scoringCommand(() -> coralIntake.runningLowAmps());
    }


    /**
     * kind of borked, returns command to move the elevator down
     * @return
     */    
    public Command moveDownCommand() {
        return Commands.sequence(
            moveToState(STOW_DOWN), 
            moveToState(STOW_UP),
            Commands.runOnce(() -> resetAllIncrements())
        );
    }

    public Command moveDownAutonomousCommand() {
        return Commands.sequence(
            moveToElevatorScoringLevelCommand(0)
            //moveToState(STOW_UP)
        );
    }
    
    public Command startIntakeCommand() {
        return moveToState(INTAKE);
    }

    public Command endIntakeCommand() {
        return moveToState(STOW_UP);
    }
    
    public Command incrementElevatorScoringLevelCommand() {
        // scoringLevel = Math.min(scoringLevel + 1, 4);
        // System.out.println(scoringLevel);
        // return moveToState(scoringStates[scoringLevel]);
        return Commands.sequence(
            Commands.runOnce(()->scoringLevel = Math.min(scoringLevel + 1, 4)), // correctly incremented
            //Commands.runOnce(()-> System.out.println(scoringLevel)),              // correct
            //new PrintCommand(Integer.toString(scoringLevel)),                     // incorrect, still 0
            // moveToSupplierState(()->scoringStates[getScoringLevel()])               // incorrect
            // moveToSupplierState(()->getScoringState(()-> scoringLevel))                // incorrect
            new DeferredCommand(()-> moveToState(scoringStates[scoringLevel]), Set.of(this)) // You arent supposed to use deffered but im so done with this shit so
        );
    }
    
    public Command decrementElevatorScoringLevelCommand() {
        // scoringLevel = Math.max(scoringLevel - 1, 0);

        // Commands.runOnce(()-> scoringLevel = 1);
        // return moveToState(scoringStates[scoringLevel]);
        return Commands.sequence(
            Commands.runOnce(()->scoringLevel = Math.max(scoringLevel - 1, 0)),
            new DeferredCommand(()-> moveToState(scoringStates[scoringLevel]), Set.of(this))
        );
    }
    
    public Command incrementDeAlgaeifyLevelCommand() {
        //deAlgaeifyLevel = Math.min(deAlgaeifyLevel + 1, 2);
        // return moveToState(deAlgaeifyStates[deAlgaeifyLevel]);
        return Commands.sequence(
            Commands.runOnce(()->deAlgaeifyLevel = Math.min(deAlgaeifyLevel + 1, 2)),
            new DeferredCommand(()-> moveToState(deAlgaeifyStates[deAlgaeifyLevel]), Set.of(this))
        );
    }

    public Command decrementDeAlgaeifyLevelCommand() {
        // deAlgaeifyLevel = Math.max(deAlgaeifyLevel - 1, 0);
        // return moveToState(deAlgaeifyStates[deAlgaeifyLevel]);
        return Commands.sequence(
            Commands.runOnce(()-> deAlgaeifyLevel = Math.max(deAlgaeifyLevel - 1, 0)),
            new DeferredCommand(()-> moveToState(deAlgaeifyStates[deAlgaeifyLevel]), Set.of(this))
        );
    }

    public Command moveToElevatorScoringLevelCommand(int scoringLevel) {
        // System.out.println(scoringLevel);
        // this.scoringLevel = scoringLevel;
        // return moveToState(scoringStates[scoringLevel]);
        return Commands.sequence(
            Commands.runOnce(()-> this.scoringLevel = scoringLevel),
            new DeferredCommand(()-> moveToState(scoringStates[scoringLevel]), Set.of(this))
        );
        
    }

    public Command moveToDeAlgaeifyLevelCommand(int deAlgaeifyLevel) {
        // System.out.println(deAlgaeifyLevel);
        // this.deAlgaeifyLevel = deAlgaeifyLevel;
        // return moveToState(deAlgaeifyStates[deAlgaeifyLevel]);
        return Commands.sequence(
            Commands.runOnce(()-> this.deAlgaeifyLevel = deAlgaeifyLevel),
            new DeferredCommand(()-> moveToState(deAlgaeifyStates[deAlgaeifyLevel]), Set.of(this))
        );
    }

    /**
     * kind of borked, schedules a command to move the elevator down
     * @return
     */    
    public void moveDown() {
        moveDownCommand().schedule();
    }

    public void startIntake() {
        startIntakeCommand().schedule();
    }

    public void endIntake() {
        endIntakeCommand().schedule();
    }

    public void incrementElevatorScoringLevel() {
        incrementElevatorScoringLevelCommand().schedule();
    }

    public boolean decrementElevatorScoringLevel() {
        decrementElevatorScoringLevelCommand().schedule();
        return scoringLevel ==  0;
    }

    public void incrementDeAlgaeifyLevel() {
        incrementDeAlgaeifyLevelCommand().schedule();;
    }

    public boolean decrementDeAlgaeifyLevel() {
        decrementDeAlgaeifyLevelCommand().schedule();
        return deAlgaeifyLevel == 0; //It's useful for the player state Machine
    }

    public void moveToElevatorScoringLevel(int scoringLevel) {
        moveToElevatorScoringLevelCommand(scoringLevel);
    }

    public void moveToDeAlgaeifyLevel(int deAlgaeifyLevel) {
        moveToDeAlgaeifyLevelCommand(deAlgaeifyLevel);
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

    public Command overrideHasCoral() {
        return Commands.runOnce(()->coralIntake.hasCoralOverride(true));
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("scoring level", scoringLevel);
        SmartDashboard.putNumber("dealgae level", deAlgaeifyLevel);
    }


}
