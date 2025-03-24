package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.subsystems.Elevator;

public class CoralIntake extends SubsystemBase {
    private final DigitalInput IRsensor = new DigitalInput(4); // false = broken

    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder altEncoder = armMotor.getAlternateEncoder();
    
    private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    private final int GEARBOX_RATIO = 15; // this is the sprocket gear ratio now
    private final int SPROCKET_RATIO = 3;

    private double setPoint = 0.0;
    private double velocity = 0.0;
    public boolean hasCoral = false;
    
    private final TrapezoidProfile.Constraints ARM_ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(5000.0/15*0.1, 5);
    private final ProfiledPIDController armController = new ProfiledPIDController(10, 0, 0, ARM_ROTATION_CONSTRAINTS);

    public CoralIntake() {

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
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(3, 0.0, 0.5)
            .outputRange(-1, 1) //what does this do??
            ;
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController.setIntegratorRange(-0.5, 0.5);
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
        this.velocity = velocity;
        rollerMotor.set(velocity);
    }

    public void moveRollerPosition(double distance) {
        rollerMotorController.setReference(distance, ControlType.kPosition);
    }

    private boolean getRollerIsNearPosition(double targetPosition) {
        return Math.abs(rollerEncoder.getPosition()-targetPosition) < .05;
    }

    public double getRollerEncoderPosition() {
        return rollerEncoder.getPosition();
    }

    public Command primeCoralForL4() {
        return Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition()-3), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition()-3));
    }

    public Command retractCoralFromL4() {
        return Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition()+5), this);
    }

    

    public Command jiggleIntakeLol(DoubleSupplier currentRollerEncoderPosition) {
        double startDistance = currentRollerEncoderPosition.getAsDouble();
        double jiggleStartDistance = -2 + startDistance;
        double jiggleDistance = 1;
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("running jiggle")),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition()-1), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition()-1)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() + jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() + jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() - jiggleDistance), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() - jiggleDistance)),
            Commands.runOnce(()-> moveRollerPosition(getRollerEncoderPosition() +1 ), this).until(() -> getRollerIsNearPosition(getRollerEncoderPosition() +1 )),
            Commands.runOnce(() -> System.out.println("ending jiggle"))
        );
    }
    
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

    public boolean getIsNearSetPoint() {
        double tolerance = 1; // in encoder rotations
        double currPosition = armEncoder.getPosition();
        double targetPosition = setPoint*SPROCKET_RATIO*GEARBOX_RATIO; 
        // System.out.print("coralIsNearSetPoint: ");
        // System.out.println(Math.abs(currPosition-targetPosition));
        return Math.abs(currPosition-targetPosition) < tolerance;
    }

    /**
     * Marked for Deprecation
     */    
    // public boolean getIsNearZero() {
    //     double tolerance = 1; // in encoder rotations
    //     double currPosition = armEncoder.getPosition(); 
    //     return Math.abs(currPosition) < tolerance;
    // }

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

    /**
     * Move the Coral Intake to the setPoint and return when finished
     * 
     * @param setPoint setpoint to move the arm to
     */    
    // public Command moveToSetPoint(double setPoint) {
    //     if (setPoint == this.setPoint) return Commands.none();
    //     return Commands.runOnce(()-> setSetPoint(setPoint), this).until(() -> getIsNearSetPoint());
    // }
    public Command moveToSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return Commands.none();
        return new FunctionalCommand(
            () -> {
                setSetPoint(setPoint);
            }, 
            null, 
            null,
            () -> getIsNearSetPoint(), 
            this);
        
    }

    @Override
    public void periodic() {
        // ill change this later
        // edit, i changed it
        //armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl);//MAXMotionPositionControl
        armMotor.setVoltage(armController.calculate(altEncoder.getPosition(), setPoint*SPROCKET_RATIO));
        SmartDashboard.putNumber("applied voltage", armController.calculate(altEncoder.getPosition(), setPoint*SPROCKET_RATIO));
        hasCoral = !IRsensor.get(); //!

        SmartDashboard.putBoolean("is at setpoint",getIsNearSetPoint());
        //SmartDashboard.putBoolean("arm is not near zero", !getIsNearZero());
        SmartDashboard.putNumber("arm setPoint", setPoint*SPROCKET_RATIO);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("arm alt encoder", altEncoder.getPosition());
        SmartDashboard.putNumber("roller amps", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasCoral", hasCoral);
        SmartDashboard.putNumber("getPositionError", armController.getPositionError());
                
        
    }
    

}
