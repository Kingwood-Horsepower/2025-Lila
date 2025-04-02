package frc.robot;

public class CoralAndElevatorState {
    public final double elevatorPosition;
    //public final double coralPrePosition;
    public final double coralEndPosition;
    public final double runRollers;
    public CoralAndElevatorState canComeFrom = null;
    public CoralAndElevatorState canGoTo = null;

    /**
     * State that the Coral and Elevator Subsystem will be commanded to move to
     * 
     * @param elevatorPosition level of the elevator in inches
     * @param coralPosition position that the coral intake will end at
     */
    public CoralAndElevatorState(double elevatorPosition, double coralPosition) {
        this.elevatorPosition = elevatorPosition;
        this.coralEndPosition = coralPosition;
        this.runRollers = 0;
    }

    /**
     * State that the Coral and Elevator Subsystem will be commanded to move to
     * 
     * @param elevatorLevel level of the elevator in inches
     * @param coralEndPosition position that the coral intake will end at
     * @param runRollers speed of the coral rollers, only use this for de-algaeify states
     */
    public CoralAndElevatorState(double elevatorPosition, double coralEndPosition, double runRollers) {
        this.elevatorPosition = elevatorPosition;
        this.coralEndPosition = coralEndPosition;
        this.runRollers = runRollers;
    }
    
    // /**
    //  * State that the Coral and Elevator Subsystem will be commanded to move to
    //  * 
    //  * @param elevatorLevel level of the elevator in inches
    //  * @param coralPrePosition position that the coral intake will move to before the elevator moves 
    //  * @param coralEndPosition position that the coral intake will end at
    //  * @param runRollers speed of the coral rollers, only use this for de-algaeify states
    //  * @param canComeFrom the previous elevator state that this state can follow
    //  * @param canGoTo the next elevator state than can follow after this state
    //  */
    // public CoralAndElevatorState(double elevatorPosition, double coralEndPosition, double runRollers, 
    //                              CoralAndElevatorState canComeFrom, CoralAndElevatorState canGoTo) {
    //     this.elevatorPosition = elevatorPosition;
    //     this.coralEndPosition = coralEndPosition;
    //     this.runRollers = runRollers;
    //     this.canComeFrom = canComeFrom;
    //     this.canGoTo = canGoTo;
    // }

    @Override
    public String toString() {
        return "e: " + String.valueOf(elevatorPosition) +  " cEnd: " + String.valueOf(coralEndPosition) + " r: " + String.valueOf(runRollers);
    }
    
    





}
