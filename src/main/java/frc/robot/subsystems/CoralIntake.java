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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.subsystems.Elevator;

public class CoralIntake extends SubsystemBase {
    private final DigitalInput IRsensor = new DigitalInput(9); // false = broken
    //private final Elevator elevator;

    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder altEncoder = armMotor.getAlternateEncoder();
    
    //look at revlib for arm motor: getForwardLimitSwitch
    //look at revlib for intake motor: getOutputCurrent

    private double setPoint = 0.0;
    private double velocity = 0.0;
    public boolean hasCoral = false;

    private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    private final int GEARBOX_RATIO = 15; // this is the sprocket gear ratio now
    private final int SPROCKET_RATIO = 3;

    
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
            .inverted(false);
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
        double tolerance = .1; // in encoder rotations
        double currPosition = altEncoder.getPosition();
        double targetPosition = setPoint*SPROCKET_RATIO; 
        return Math.abs(currPosition-targetPosition) < tolerance;
    }

    public boolean getIsNearZero() {
        double tolerance = .1; // in encoder rotations
        double currPosition = altEncoder.getPosition(); 
        return Math.abs(currPosition) < tolerance;
    }

    public boolean runningHighAmps(){
        double threshold = 10;
        return (rollerMotor.getOutputCurrent() > threshold);
    }

    @Override
    public void periodic() {
        // ill change this later
        //armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl);//MAXMotionPositionControl
        armMotor.setVoltage(armController.calculate(altEncoder.getPosition(), setPoint*SPROCKET_RATIO));
        SmartDashboard.putNumber("applied voltage", armController.calculate(altEncoder.getPosition(), setPoint*SPROCKET_RATIO));
        hasCoral = !IRsensor.get(); 

        SmartDashboard.putBoolean("is at setpoint",getIsNearSetPoint());
        SmartDashboard.putBoolean("arm is not near zero", !getIsNearZero());
        SmartDashboard.putNumber("arm setPoint", setPoint*SPROCKET_RATIO);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("arm alt encoder", altEncoder.getPosition());
        SmartDashboard.putNumber("roller amps", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasCoral", hasCoral);
        SmartDashboard.putNumber("getPositionError", armController.getPositionError());
                
        
    }
    public boolean hasCoral()
    {
        return hasCoral;
    }

}
