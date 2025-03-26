package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.google.gson.internal.TroubleshootingGuide;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Elevator extends SubsystemBase{

    
    DigitalInput limitSwitch = new DigitalInput(8);
    //DigitalInput IRZero = new DigitalInput(8);

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
    

    private double setPoint = 0.0;
    private boolean isZerod = true;
    

    private final TrapezoidProfile.Constraints ELEVATOR_MOTOR_ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(2000, 1000);
    private final ProfiledPIDController elevatorController = new ProfiledPIDController(.6, 0, 0, ELEVATOR_MOTOR_ROTATION_CONSTRAINTS);

    //sim
    Mechanism2d mech = new Mechanism2d(1, 6);
    MechanismRoot2d rootMech = mech.getRoot("base", 0.5, 0);
    DCMotor elevatorDCMotor = DCMotor.getNEO(1);
    private final SparkMaxSim leadMotorSim = new SparkMaxSim(leadMotor, elevatorDCMotor);

    private final ElevatorSim elevatorSim = new ElevatorSim(
        elevatorDCMotor, 12, 9, 0.0152, 0, 1.25, true, 0, 0.01, 0);
    private final MechanismLigament2d elevatorMech2d =
      rootMech.append(
          new MechanismLigament2d("elevator", 0.1, 90)
          );
    private final SparkRelativeEncoderSim leadEncoderSim = leadMotorSim.getRelativeEncoderSim();
    

    public Elevator() {
        leadMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            
            
            ; // <- this semicolon is VERY important
        leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .follow(leadMotor, true);
        followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint double that represents distance of the middle stage to its bottom 
     */
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }


    public double getLeadEncoderPosition() {
        double pos = -1;
        if(Robot.isSimulation()) pos = leadEncoderSim.getPosition();
        pos = leadEncoder.getPosition();
        return pos;
    }

    public boolean getIsNearSetPoint() {
        double tolerance = 2;  // in encoder rotations
        double currPosition = getLeadEncoderPosition(); 
        double targetPosition = setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS;
        return Math.abs(currPosition-targetPosition) < tolerance;
    }

    public boolean getIsLimitSwitchZerod() {
        return isZerod;
    }

    public void resetEncoders() {
        leadEncoder.setPosition(0.0);
        followEncoder.setPosition(0.0);
    }

    /**
     * Move the Elevator to the setPoint and return when finished
     * 
     * @param setPoint setpoint to move the arm to
     */    
    public Command moveToSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return Commands.none();
        return Commands.runOnce(()-> setSetPoint(setPoint), this);//.until(() -> getIsNearSetPoint());
    }

    @Override
    public void periodic() {

        //leadMotorController.setReference(setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS, ControlType.kMAXMotionPositionControl);
        leadMotor.setVoltage(elevatorController.calculate(getLeadEncoderPosition(), setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS));
        isZerod = !limitSwitch.get();


        SmartDashboard.putNumber("lead elevator encoder", leadEncoder.getPosition());
        SmartDashboard.putNumber("follow elevator encoder", followEncoder.getPosition());
        SmartDashboard.putNumber("elevator setpoint", setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS);
        //SmartDashboard.putNumber("elevator level", getElevatorLevel());
        SmartDashboard.putBoolean("is zeroed limitwsitch", isZerod);
        //SmartDashboard.putBoolean("elevator is near zero", getIsNearZero());
        SmartDashboard.putBoolean("elevator is near setpoint", getIsNearSetPoint());
        SmartDashboard.putNumber("elevator motor current", leadMotor.getOutputCurrent());

    }

    @Override 
    public void simulationPeriodic() {
        
        elevatorSim.update(0.02);

        // setting the motor sim to the mechanism's current calculated velocity
        leadMotorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
        // simulating the power draw from the battery by this mechanism
        RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        // calculate the mechanism's new velocity with the motor sim
        elevatorSim.setInput(leadMotorSim.getVelocity() * RobotController.getBatteryVoltage());
        // set the encododer sim 
        leadEncoderSim.setPosition(leadMotorSim.getPosition());
        // modify the mechanism
        elevatorMech2d.setLength(1+elevatorSim.getPositionMeters());
        //System.out.println(leadMotorSim.getPosition());

        SmartDashboard.putData("elevator please", mech);
        
    }
}