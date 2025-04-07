package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CoralIntakeConstants.*;

public class CoralIntake extends SubsystemBase {
    // create objects for sensors and motors
    private final DigitalInput IRsensor = new DigitalInput(3); // false = broken

    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder altEncoder = armMotor.getAlternateEncoder();
    
    private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    // private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    // private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    // velocity and acceleration constraints of the PID controller below. 
    // The PID controller is what calculates wrist speed in periodic()
    private final TrapezoidProfile.Constraints ARM_ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(5000.0/15*0.1, 7);
    private final ProfiledPIDController armController = new ProfiledPIDController(10, 0, 0, ARM_ROTATION_CONSTRAINTS);
    
    private final Debouncer zeroingDebouncer = new Debouncer(0.8, DebounceType.kRising); // zero after 4 schedular loop runs

    private boolean hasCoralOverrideTrue = false;
    private boolean hasCoralOverrideFalse = false;
    // the setPoint of the wrist. The wrist is driven to this value on every run of periodic()
    private double setPoint = 0.0;
    private boolean hasCoral = false;

    public CoralIntake() {
        //configure sparkmax objects
        armMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        armMotorConfig
            .alternateEncoder
            .inverted(true)
            .countsPerRevolution(8192);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        rollerMotorConfig
            .smartCurrentLimit(38)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
            // // this sparkmax uses simple pid control for the rollers position of the coral intake, 
            // // its intended use was in jiggling, which was never used
            // .closedLoop
            // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // .pid(1.5, 0.0, 0)
            // .outputRange(-1, 1);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint rotations of the big sprocket
     */
    public void setSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return;
        this.setPoint = setPoint;
    }

    public void setRollerVelocity(double velocity) {
        rollerMotor.set(velocity);
    }

    // // used to move the coral a distance, was used for priming and jiggling but they were never used
    // public void moveRollerPosition(double distance) {
    //     rollerMotorController.setReference(distance, ControlType.kPosition);
    // }

    // private boolean getRollerIsNearPosition(double targetPosition) {
    //     return Math.abs(rollerEncoder.getPosition()-targetPosition) < .2;
    // }

    // public double getRollerEncoderPosition() {
    //     return rollerEncoder.getPosition();
    // }

    // public void primeCoralForL4() {
    //     moveRollerPosition(getRollerEncoderPosition()-.7);
    //     System.out.println("primed");
    // }

    // public void retractCoralFromL4() {
    //     moveRollerPosition(getRollerEncoderPosition()+1.2);
    // }

    // public Command jiggleIntakeLol(DoubleSupplier currentRollerEncoderPosition) {
    //     double startDistance = currentRollerEncoderPosition.getAsDouble();
    //     double jiggleDistance = 1;
    //     return Commands.sequence(
    //         Commands.runOnce(() -> System.out.println("running jiggle")),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition()-1), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition()-1)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
    //         Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() +1 ), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() +1 )),
    //         Commands.runOnce(() -> System.out.println("ending jiggle"))
    //     );
    // }
    
    /**
     * Command arm to setpoint and run intake
     * 
     * @param setPoint rotations of the big sprocket, ~ 0, 0.3
     * @param velocity velocity of the intake motor,   -1, 1 
     */
    public void runIntake(double setPoint, double velocity) {
        setSetPoint(setPoint);
        setRollerVelocity(velocity);
    }

    /**
     * Determines if the coralWrist is near (2 motor rotations of tolerance) the current setPoint
     */    
    public boolean getIsNearSetPoint() {
        double tolerance = .01; // in encoder rotations
        double currPosition = altEncoder.getPosition();
        double targetPosition = setPoint*SPROCKET_RATIO; 
        return Math.abs(currPosition-targetPosition) < tolerance;
    }

    public boolean runningHighAmps(){
        double threshold = 10;
        return (rollerMotor.getOutputCurrent() > threshold);
    }

    public boolean runningLowAmps(){
        double threshold = 35;
        return (rollerMotor.getOutputCurrent() < threshold);
    }
    
    public boolean hasCoral()
    {
        return hasCoral;
    }

    // /**
    //  * Move the Coral Intake to the setPoint and return when finished
    //  * 
    //  * @param setPoint setpoint to move the arm to
    //  */    

    // public Command moveUntilAtSetPoint(double setPoint) {
    //     if (setPoint == this.setPoint) return Commands.none();
    //     return new FunctionalCommand(
    //         () -> {
    //             setSetPoint(setPoint);
    //         }, 
    //         ()->{}, 
    //         t->{},
    //         () -> getIsNearSetPoint(), 
    //         this);
        
    // }

    public void overrideCoralTrue() {
        hasCoralOverrideTrue = true;
        System.out.println("Coral override: TRUE");
    }

    public void overrideCoralFalse() {
        hasCoralOverrideTrue = false;
        System.out.println("Coral override: FALSE");
    }

 
    // private Timer zeroStallingTimer = new Timer();
    // private boolean zeroed = false;
    
    // // zero the elevator by hitting the bottom
    // public Command zeroCoralElevatorCommand() {
    //     boolean isStalling = false;
    //     return new FunctionalCommand(
    //         ()->{
    //             zeroStallingTimer.reset();
    //             currentlyZeroing = true; // whenever we are zeroing, we prevent the periodic setting of the PID Controller
    //             armMotor.set(0.2);
    //             System.out.println("what");
                
    //         }, 
    //         ()->{
    //             // if we have stalled, and the stall timer isnt running, start the stall timer
    //             if (Math.abs(altEncoder.getVelocity()) < 1 && !zeroStallingTimer.isRunning()) zeroStallingTimer.start();
    //             else if (Math.abs(altEncoder.getVelocity()) < 1) {} // do nothing 
    //             // if we are not stalled, reset the timer.
    //             else zeroStallingTimer.reset();
    //         }, 
    //         onEnd -> {
    //             armMotor.set(0);
    //             altEncoder.setPosition(CORAL_ZERO_POINT);
    //             currentlyZeroing = false;
    //             zeroed = true;
    //         }, 
    //         ()->(zeroStallingTimer.get()) > .08, 
    //         // the motor has not moved - stalled - for .5 second, 
    //         // (the stall timer is greater than .5),
    //         // then we hit the bottom bar
    //         this);
    // }

    private boolean currentlyZeroing = false;

    // zero the elevator by hitting the bottom
    public Command zeroCoralElevatorCommand() {
        return new FunctionalCommand(
            ()->{
                currentlyZeroing = true;
                armMotor.set(0.2);
            },
            ()->{}, 
            onEnd -> {  // the name "onEnd" doesnt actually exist. 
                        // you can literally name the parameter of the consumer to anything and it will still work
                armMotor.set(0);
                altEncoder.setPosition(CORAL_ZERO_POINT);
                currentlyZeroing = false;
            },  
            // an alt encoder velocity of 1 is the threshold that is considered stalling. This was determined experimentally
            // meaning i just guessed a value, then tuned that value to be as low as possible while the function still worked
            ()->zeroingDebouncer.calculate(altEncoder.getVelocity()<1),
            this);
    }

    @Override
    public void periodic() {

        if(!currentlyZeroing) armMotor.setVoltage(armController.calculate(altEncoder.getPosition(), setPoint*SPROCKET_RATIO));

        //hasCoral = !IRsensor.get()||hasCoralOverrideTrue; //!
        hasCoral = hasCoralOverrideFalse ? false : !IRsensor.get() ||hasCoralOverrideTrue;

        SmartDashboard.putBoolean("is at setpoint",getIsNearSetPoint());
        //SmartDashboard.putBoolean("arm is not near zero", !getIsNearZero());
        SmartDashboard.putNumber("arm setPoint", setPoint*SPROCKET_RATIO);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("arm alt encoder", altEncoder.getPosition());
        SmartDashboard.putNumber("roller amps", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasCoral", hasCoral);
        SmartDashboard.putBoolean("isOverrideTrue", hasCoralOverrideTrue);
        SmartDashboard.putBoolean("isOverrideFalse", hasCoralOverrideFalse);
        SmartDashboard.putNumber("getPositionError", armController.getPositionError());
        //SmartDashboard.putNumber("zeroStallTimer", zeroStallingTimer.get());
        SmartDashboard.putNumber("armVelocity", altEncoder.getVelocity());
        SmartDashboard.putBoolean("timer should be runnning", Math.abs(altEncoder.getVelocity()) < 1 );
        SmartDashboard.putBoolean("curr zeoring", currentlyZeroing);
        //SmartDashboard.putBoolean("coral intake zeroed", zeroed);
                
        
    }
    

}
