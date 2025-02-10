package frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants.OperatorConstants.AutoConstants;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    
    //Joystick
    private final Joystick joy1 = new Joystick(0);

    //Motors
    private final SparkMax leftMaster;
    private final SparkMax leftSlave;
    private final SparkMax rightMaster;
    private final SparkMax rightSlave;

    //Robot's Drive
    public final DifferentialDrive m_Drive;

    //Encoders
    final Encoder leftEncoder;
    final Encoder rightEncoder;

    //NavX Sensor
    private final ADXRS450_Gyro navX = new ADXRS450_Gyro();

    private Field2d field = new Field2d();
    private Pose2d currentPose;

    //Datas we will use
    private double leftRoad;
    private double rightRoad;

    private double leftSimRoad;
    private double rightSimRoad;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TrackWidthMeters);
    private final PIDController xPidController = new PIDController(AutoConstants.PXController, AutoConstants.IXController, AutoConstants.DXController);
    private final PIDController yawPidController = new PIDController(AutoConstants.PYawController, AutoConstants.IYawController, AutoConstants.DYawController);

    //Supresses useless warnings
    @SuppressWarnings("removal")
    public DriveSubsystem() {
        resetSensors();

        //Defining motors
        leftMaster = new SparkMax(DriveConstants.LeftMotor1Port, MotorType.kBrushed);
        leftSlave = new SparkMax(DriveConstants.LeftMotor2Port, MotorType.kBrushed);
        rightMaster = new SparkMax(DriveConstants.RightMotor1Port, MotorType.kBrushed);
        rightSlave = new SparkMax(DriveConstants.RightMotor2Port, MotorType.kBrushed);

        //Defining encoders
        leftEncoder = new Encoder(
            DriveConstants.LeftEncoderPorts[0],
            DriveConstants.LeftEncoderPorts[1],
            DriveConstants.LeftEncoderInverted); 
        rightEncoder = new Encoder(
            DriveConstants.RightEncoderPorts[0],
            DriveConstants.RightEncoderPorts[1],
            DriveConstants.RightEncoderInverted); 

        //Setting distance per pulse
        leftEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.EncoderDistancePerPulse);

        //Odometry for tracking robot's location
        odometry = 
        new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            leftEncoder.getDistance(), 
            rightEncoder.getDistance());

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        leftRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftEncoder.get());
        rightRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightEncoder.get());

        final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);
        final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

        m_Drive = new DifferentialDrive(leftMotors, rightMotors);

        leftMaster.setCANTimeout(250);
        leftSlave.setCANTimeout(250);
        rightMaster.setCANTimeout(250);
        rightSlave.setCANTimeout(250);

        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(0);

        rightMaster.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        leftMaster.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(leftMaster);
        leftSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(rightMaster);
        rightSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    //Updates robot's location on the field
    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            leftEncoder.getDistance(),
            rightEncoder.getDistance());
        field.setRobotPose(getPose());
        //System.out.print("RED :"+getPose().getRotation().getDegrees());
        //System.out.println("BLUE :"+getAngleCalculationForBlue());
        SmartDashboard.putNumber("Axis 1", joy1.getRawAxis(1));
        SmartDashboard.putNumber("Axis 4", joy1.getRawAxis(4));
    }

    public Command driveArcade(
        DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        return Commands.run(
            () -> m_Drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            Rotation2d.fromDegrees(pose.getRotation().getDegrees()),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            pose);
    }

    public double getLeftRoad() {
        return leftRoad;
    }
    public double getRightRoad() {
        return rightRoad;
    }
    public double getLeftSimRoad() {
        return leftSimRoad;
    }
    public double getRightSimRoad() {
        return rightSimRoad;
    }
 
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    public void resetSensors() {
        resetNavX();
    }
    public void resetNavX() {
        navX.reset();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }
    public double getTurnRate() {
        return -navX.getRate();
    }
    public double getYaw() {
        return navX.getAngle();
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public double getAngleCalculationForBlue(){
        return (getHeading() + 360) % 360;
    }
}