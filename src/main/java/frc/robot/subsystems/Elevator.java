package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.math.util.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    DigitalInput magneticLimitSwitch = new DigitalInput(1);
    DigitalInput IRZero = new DigitalInput(8);

    private final int leadMotorID = 21;
    private final int followMotorID = 19;

    private final SparkMax leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
    private final SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController leadMotorController = leadMotor.getClosedLoopController();
    private final RelativeEncoder leadEncoder = leadMotor.getEncoder();

    private final SparkMax followMotor = new SparkMax(followMotorID, MotorType.kBrushless);
    private final SparkMaxConfig followMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder followEncoder = followMotor.getEncoder();

    private final int ELEVATOR_GEAR_RATIO = 12;
    private final double ELEVATOR_SPROCKET_CIRCUMFERENCE = 1.281*Math.PI;
    private final double ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS = ELEVATOR_GEAR_RATIO/ELEVATOR_SPROCKET_CIRCUMFERENCE;
    

    private final double[] ELEVATOR_LEVELS = {
        ELEVATOR_HOME_INCHES,
        ELEVATOR_L1_INCHES,
        ELEVATOR_L2_INCHES,
        ELEVATOR_L3_INCHES,
        ELEVATOR_MAX_INCHES,
        ELEVATOR_MAX_INCHES,
    };

    private double setPoint = 0.0;
    private int elevatorLevel = 0;
    private boolean isZerod = true;
    
    private double ElevatorMaxExtensionInches = ELEVATOR_MAX_INCHES;
    //private String maxExtensionPreferenceKey = "Elevator Max Extension Inches";

    public Elevator() {
        leadMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .pid(0.1, 0, 0)

            .outputRange(-1, 1)
            .velocityFF(0) // calculated using recalc
            .maxMotion
            //idk if i want to use the units library on top of the units math util, its very verbose
            .maxVelocity(3500) // takes an rpm 

            .maxAcceleration(5000) // takes an rpm/s
            .allowedClosedLoopError(0.6)

            ; // <- this semicolon is important
        leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .follow(leadMotor, true);
        followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Preferences.initDouble(maxExtensionPreferenceKey, ElevatorMaxExtensionInches);
    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint double that represents distance of the middle stage to its bottom 
     */
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public int getElevatorLevel() {
        return elevatorLevel;
    }

    public void incrementElevatorLevel(){
        if (elevatorLevel == 4) elevatorLevel = 0;
        else elevatorLevel += 1;
        SmartDashboard.putNumber("elevator level", elevatorLevel);
        System.out.println(elevatorLevel);
    }
    public void decrementElevatorLevel(){
        if (elevatorLevel == 0) elevatorLevel = 4;
        else elevatorLevel -= 1;
        SmartDashboard.putNumber("elevator level", elevatorLevel);
        System.out.println(elevatorLevel);
    }

    public void setElevatorLevel(){
        setSetPoint(ELEVATOR_LEVELS[elevatorLevel]);
    }

    public void setElevatorLevel(int level){
        elevatorLevel = level;
        setElevatorLevel();
    }

    public boolean getIsNearSetPoint() {
        double tolerance = 2;  // in encoder rotations
        double currPosition = leadEncoder.getPosition(); 
        double targetPosition = setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS;
        if ((currPosition > targetPosition - tolerance) && (currPosition < targetPosition + tolerance)) return true;
        return false;
    }

    public boolean getIsNearZero() {
        double tolerance = 5; // in encoder rotations
        double currPosition = leadEncoder.getPosition();
        if ((currPosition > -1*tolerance) && (currPosition < tolerance)) return true;
        return false;
    }

    public void zeroElevator() {
        leadEncoder.setPosition(0.0);
        followEncoder.setPosition(0.0);
    }

    public void resetEncoders() {
        leadEncoder.setPosition(0.0);
        followEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {

        leadMotorController.setReference(setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS, ControlType.kMAXMotionPositionControl);
        isZerod = !IRZero.get();

        SmartDashboard.putNumber("lead elevator encoder", leadEncoder.getPosition());
        SmartDashboard.putNumber("follow elevator encoder", followEncoder.getPosition());
        SmartDashboard.putNumber("elevator setpoint", setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS);
        //SmartDashboard.putBoolean("elevator is near zero", getIsNearZero());
        SmartDashboard.putBoolean("isZerod", isZerod);

    }

}