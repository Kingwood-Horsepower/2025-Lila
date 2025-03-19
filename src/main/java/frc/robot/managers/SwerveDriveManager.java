package frc.robot.managers;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDriveManager {
    //Data
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final double translationVelocityMult = 0.65; // Cannot be more than 1
    private final double rotVelocityMult = .75;        

    //IMPORTANT STUFF
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverController;                                                                      // max angular velocity

    //SwerveRequestes
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)// (not) Use open-loop control for drive motors
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
           .withDeadband(MaxSpeed * 0.01)
           .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
           .withDriveRequestType(DriveRequestType.Velocity);// (not) Use open-loop control for drive motors
          //.withSteerRequestType(SteerRequestType.);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();   

    private final SwerveRequest.ApplyFieldSpeeds trajectoryRequest = new SwerveRequest.ApplyFieldSpeeds()       
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);      //SwerveRequest used by the follow trajectory method;


    // SlewRaeLimiters
    private final SlewRateLimiter driveLimiterX = new SlewRateLimiter(1.3); // How fast can the robot accellerate                                                                                // and decellerate
    private final SlewRateLimiter driveLimiterY = new SlewRateLimiter(1.3);
    private final SlewRateLimiter driveLimiterRot = new SlewRateLimiter(2.6);
    private final SlewRateLimiter driveLimiterSlowRot = new SlewRateLimiter(1.1);

    //PID Controllers (For trajectory following)
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10,2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController xController = new ProfiledPIDController(7, 0, 0.2, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(7, 0, 0.2, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(10, 0, 0, THETA_CONSTRAINTS);

    //Variables
    private int inputMult =1;
    public SwerveDriveManager(CommandXboxController driverController ) {
        this.driverController = driverController;
        drivetrain.registerTelemetry(logger::telemeterize);
        setStartPose();
        //drivetrain.setDefaultCommand(setSwerveToNormalDriveCommand());
        inputMult =1;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);   
    }

    //Robot Movement functions
    public Command setSwerveToNormalDriveCommand(){
        Command command = drivetrain.applyRequest(() -> drive
            .withVelocityX(driveLimiterX.calculate(driverController.getLeftY()* getInputMult()) * translationVelocityMult
                    * MaxSpeed)
            .withVelocityY(driveLimiterY.calculate(driverController.getLeftX()* getInputMult()) * translationVelocityMult
                    * MaxSpeed)
            .withRotationalRate(driveLimiterRot.calculate(driverController.getRightX()) * -1
                                * rotVelocityMult * MaxAngularRate));
        
        command.addRequirements(drivetrain);
        return command;
    }

    public Command setSwerveToSlowDriveCommand(){
        Command command =  drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(-driverController.getLeftY()* translationVelocityMult
            * MaxSpeed  * 0.2 )
        .withVelocityY(-driverController.getLeftX() * translationVelocityMult
            * MaxSpeed * 0.2 )
        .withRotationalRate(driverController.getRightX()* -1
            * rotVelocityMult * MaxAngularRate * 0.4));

        command.addRequirements(drivetrain);
        return command;
    }

    public Command getSwerveDriveScoringCommand(){
        Command command =  drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(-driverController.getLeftY()* translationVelocityMult
            * MaxSpeed  * 0.2 )
        .withVelocityY(0)
        .withRotationalRate(0));

        command.addRequirements(drivetrain);
        return command;
    }

    public void setSwerveDriveChassisSpeeds(ChassisSpeeds speeds){
        drivetrain.setControl(trajectoryRequest.withSpeeds(speeds));
    }
    public void resetAutoTrajectory(){
        xController.reset(getRobotPose().getX());
        yController.reset(getRobotPose().getY());
        thetaController.reset(getRobotPose().getRotation().getRadians());
    }
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getRobotPose();
        //System.out.println("est: " +pose.getTranslation());
        //System.out.println("right: " + Double.toString(sample.x) + "   y: " + Double.toString(sample.y));
        System.out.println("Xdiff: " + (pose.getTranslation().getX()-sample.x));

        //WE NEED TO APPLY PID AND FEED FORWARD
        //use sample.x, sample.y, and sample.heading for where the robot is supposed to be (and confront it with the pose)
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        // ChassisSpeeds speeds = new ChassisSpeeds(
        //     sample.vx,
        //     sample.vy,
        //     sample.omega 
        // );
        // Apply the generated speeds
        drivetrain.setControl(trajectoryRequest.withSpeeds(speeds));
    }


    //Utility Functions
    public void stopRobot(){
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        // Apply the generated speeds
        drivetrain.setControl(trajectoryRequest.withSpeeds(speeds));
        //drivetrain.setControl(brake);

    }
    public Pose2d getRobotPose()
    {
        return drivetrain.getState().Pose;
    }
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
    public void setStartPose() {
        var startPose = new Pose2d(AutoConstants.getStartingPosition(), new Rotation2d(Math.PI));
        drivetrain.resetPose(startPose);
    }
    public void resetPose(Pose2d newPose2d) {
        drivetrain.resetPose(newPose2d);
    }
    public ChassisSpeeds getRobotSpeeds()
    {
        return drivetrain.getState().Speeds;
    }

    //Vision
    public void setVisionTrust(Matrix<N3, N1> matrix){
        drivetrain.setVisionMeasurementStdDevs(matrix);
    }
    public void addVisionMeasurement(Pose2d pose, double timestamp){
        drivetrain.addVisionMeasurement(pose, timestamp);
    }   



    //Controls inversion
    private int getInputMult(){
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        if(isBlue){
            return -1 * inputMult;
        }else{
            return 1 * inputMult;
        }
    }
    public void invertControls(){
        inputMult = inputMult * -1;
    }

    


    //Data Logging
    public void sendSwerveData() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            
            builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);
            
            builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().RawHeading.getRadians(), null);
            }
            });
    }



}
