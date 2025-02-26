package frc.robot.subsystems;

//Importing libraries
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    //Defining joystick and motors
    XboxController xboxController = new XboxController(0);

    private final TalonFX leftMaster = new TalonFX(DriveConstants.LeftMotor1Port);
    private final TalonFX leftSlave = new TalonFX(DriveConstants.LeftMotor2Port);
    
    private final TalonFX rightMaster = new TalonFX(DriveConstants.RightMotor1Port);
    private final TalonFX rightSlave = new TalonFX(DriveConstants.RightMotor2Port);

    //Defining the differential drive
    private final DifferentialDrive m_drive =
        new DifferentialDrive(leftMaster::set, rightMaster::set);
    
    //Defining the encoders and gyro
    private final Encoder leftEncoder = 
        new Encoder(
            DriveConstants.LeftEncoderPorts[0], 
            DriveConstants.LeftEncoderPorts[1],
            DriveConstants.LeftEncoderReversed);

    private final Encoder rightEncoder = 
        new Encoder(
            DriveConstants.RightEncoderPorts[0], 
            DriveConstants.RightEncoderPorts[1],
            DriveConstants.RightEncoderReversed);
    
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    double outputYcoeffector = -1;

    //Creating odometry
    private final DifferentialDriveOdometry odometry;

    //Defining variables that will be used to calculate the robot's position/angle/velocity
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.TrackwidthMeters);
    private final PIDController xPidController = new PIDController(AutoConstants.PXController, AutoConstants.IXController, AutoConstants.DXController);
    private final PIDController yawPidController = new PIDController(AutoConstants.PYawController, AutoConstants.IYawController, AutoConstants.DYawController);

    //Defining the simulation variables
    public Pose2d pose;
    public DifferentialDrivetrainSim driveTrainSim;
    public EncoderSim leftEncoderSim;
    public EncoderSim rightEncoderSim;
    private final Field2d field;
    private final ADXRS450_GyroSim gyroSim;
    double road;

    double currentAngle = 0;

    @SuppressWarnings("removal")
    public DriveSubsystem() {
        
        //Adding the motors to the dashboard
        SendableRegistry.addChild(m_drive, leftMaster);
        SendableRegistry.addChild(m_drive, rightMaster);

        //Setting the followers
        leftSlave.setControl(new Follower(DriveConstants.LeftMotor1Port, false));
        rightSlave.setControl(new Follower(DriveConstants.RightMotor1Port, false));

        //Inverting right motors
        rightMaster.setInverted(true);

        //Configuring the encoders
        leftEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);

        resetEncoders();
        //Defining the odometry
        odometry = 
            new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

        //Defines simulation variables if the robot is in simulation mode
        if (RobotBase.isSimulation()) {
            driveTrainSim = new DifferentialDrivetrainSim(
                DriveConstants.DrivetrainPlant, 
                DriveConstants.DriveGearbox, 
                DriveConstants.DriveGearing, 
                DriveConstants.TrackwidthMeters, 
                DriveConstants.WheelDiameterMeters / 2.0, 
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);

            field = new Field2d();
            SmartDashboard.putData("Field", field);
                
            
        //If not in simulation mode, the simulation variables are set to null
        } else {
            leftEncoderSim = null;
            rightEncoderSim = null;
            gyroSim = null;
            field = null;
        }
  }

    @Override
    //Runs the code inside every 20ms
    public void periodic() {
        //Updates the odometry
        pose = odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            leftEncoder.getDistance(), 
            rightEncoder.getDistance());

        //Updates robot's pose on the field
        field.setRobotPose(pose);
        road = leftEncoder.get() * DriveConstants.EncoderDistancePerPulse / DriveConstants.DriveGearing * 6.91;
    }

    @Override
    public void simulationPeriodic() {
        //Sets simulation motors' inputs to motors'
        driveTrainSim.setInputs(
            leftMaster.get() * RobotController.getBatteryVoltage(), 
            rightMaster.get() * RobotController.getBatteryVoltage());
        driveTrainSim.update(0.02);

        //Sets simulation encoders' distance and rate to encoders'
        leftEncoderSim.setDistance(driveTrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveTrainSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(driveTrainSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveTrainSim.getRightVelocityMetersPerSecond());

        //Sets simulation gyro's angle
        gyroSim.setAngle(-driveTrainSim.getHeading().getDegrees());
    }

    SlewRateLimiter outputYLimiter = new SlewRateLimiter(1);
    double outputY;
    double outputX;

    public void arcadeDrive(double forward, double rotation) {
        outputY = forward;
        outputX = rotation*0.8;
        //If RB is pressed, the robot will move slower
        if(xboxController.getRightBumper()){
            outputY = outputYLimiter.calculate(outputYcoeffector*forward*0.9);
            outputX = rotation*0.55;
        }
        
        m_drive.arcadeDrive(outputY, outputX);
    }
    
    public void drive(ChassisSpeeds speeds) {
        m_drive.feed();

        xPidController.setSetpoint(speeds.vxMetersPerSecond);
        yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

        m_drive.arcadeDrive(xPidController.calculate(getChassisSpeeds().vxMetersPerSecond), yawPidController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        m_drive.feed();
    }

    //Returns the robot's speed that calculated from wheels' speeds
    public ChassisSpeeds getChassisSpeeds(){
        return m_kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    //Returns the wheels' speeds
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    //Resets the odometry to robot's real position and angle
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        driveTrainSim.setPose(pose);
        odometry.resetPosition(
            Rotation2d.fromDegrees(pose.getRotation().getDegrees()),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            pose);
    }

    //Resets the gyro
    public void zeroHeading() {
        gyro.reset();
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    //Returns the robot's pose
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    //Resets the encoders
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
 
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.GyroReversed ? -1.0 : 1.0);
    }

    public double getRoad(){
        return road;
    }
}
