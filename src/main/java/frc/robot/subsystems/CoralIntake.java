package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.subsystems.Elevator;

public class CoralIntake extends SubsystemBase {
    private final DigitalInput IRsensor = new DigitalInput(9); // false = broken
    private final Elevator elevator;

    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder altEncoder = armMotor.getAlternateEncoder();

    private double setPoint = 0.0;
    private double velocity = 0.0;
    public boolean hasCoral = false;

    private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    private final int ARM_GEAR_RATIO = 3; // this is the sprocket gear ratio now

    public CoralIntake(Elevator elevator) {
        this.elevator = elevator;
        // continue setup
        

        armMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true) 
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(1, 0.0, 0.0)
            .outputRange(-1, 1) //what does this do??
            //.velocityFF(0) // calculated using recalc
            .maxMotion
            //idk if i want to use the units library on top of the units math util, its very verbose
            .maxVelocity(266) // takes an rpm 
            .maxAcceleration(500) // takes an rpm/s
            .allowedClosedLoopError(0.02)
            ; // <- this semicolon is important
        armMotorConfig
            .alternateEncoder
            .countsPerRevolution(8192)
            .setSparkMaxDataPortConfig()
            .inverted(true)
            ;
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        rollerMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

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
        this.velocity = velocity;
        rollerMotor.set(velocity);
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

    public void stowIntake(){

        // if i am not near zero, or not set to 0, or i have coral set to down position
        if (!elevator.getIsNearZero() || elevator.getElevatorLevel() > 0 || hasCoral) setSetPoint(.26);
        // set up
        else setSetPoint(0.0);
        setRollerVelocity(0.0);
    }

    public boolean getIsNearSetPoint() {
        double tolerance = 0.1; // in small sprocket rotations
        double currPosition = altEncoder.getPosition();
        double targetPosition = setPoint*ARM_GEAR_RATIO; 
        if ((currPosition > targetPosition - tolerance) && (currPosition < targetPosition + tolerance)) return true;
        return false;
    }

    public boolean getIsNearZero() {
        double tolerance = 0.1; // in small sprocket rotations
        double currPosition = altEncoder.getPosition(); 
        if ((currPosition > 0 - tolerance) && (currPosition < 0 + tolerance)) return true;
        return false;
    }

    public boolean runningHighAmps(){
        double threshold = 10;
        return (rollerMotor.getOutputCurrent() > threshold);
    }

    @Override
    public void periodic() {
        // ill change this later
        armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl);//MAXMotionPositionControl
        hasCoral = !IRsensor.get(); 

        SmartDashboard.putBoolean("is at setpoint",getIsNearSetPoint());
        SmartDashboard.putBoolean("arm is not near zero", !getIsNearZero());
        SmartDashboard.putNumber("arm setPoint", setPoint*ARM_GEAR_RATIO);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("arm alt encoder", altEncoder.getPosition());
        SmartDashboard.putNumber("roller amps", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasCoral", hasCoral);
                
        
    }
    public boolean hasCoral()
    {
        return hasCoral;
    }

}
