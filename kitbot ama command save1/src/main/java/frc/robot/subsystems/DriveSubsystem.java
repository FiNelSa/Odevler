package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DropperSubsystem;
import org.littletonrobotics.junction.Logger;


public class DriveSubsystem extends SubsystemBase {
    //Motors
    private final SparkMax leftMaster;
    private final SparkMax leftSlave;
    private final SparkMax rightMaster;
    private final SparkMax rightSlave;

    //Robot's Drive
    public final DifferentialDrive m_Drive;

    //JoyStick
    private Joystick joy1 = new Joystick(0);

    //Encoders
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final EncoderSim leftSimEncoder;
    private final EncoderSim rightSimEncoder;
    
    private Rotation2d gyroAngle;

    //NavX Sensor
    private final AHRS navX = new AHRS(Port.kMXP);

    //Odometry for tracking robot's location
    private final DifferentialDriveOdometry odometry;

    final DifferentialDrivetrainSim driveSim;
    private Field2d field = new Field2d();
    private Pose2d currentPose;

    //Datas we will use
    private double leftRoad;
    private double rightRoad;
    
    private double leftSimRoad;
    private double rightSimRoad;

    //Supresses useless warnings
    @SuppressWarnings("removal")
    public DriveSubsystem() {

        leftMaster = new SparkMax(0, MotorType.kBrushed);
        leftSlave = new SparkMax(1, MotorType.kBrushed);
        rightMaster = new SparkMax(2, MotorType.kBrushed);
        rightSlave = new SparkMax(3, MotorType.kBrushed);

        leftEncoder = new Encoder(0, 1); 
        rightEncoder = new Encoder(2, 3);

        leftSimEncoder = new EncoderSim(leftEncoder);
        rightSimEncoder = new EncoderSim(rightEncoder);

        leftRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftEncoder.get());
        rightRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightEncoder.get());

        leftSimRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftSimEncoder.getDistance());
        rightSimRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightSimEncoder.getDistance());

        final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);
        final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

        m_Drive = new DifferentialDrive(leftMotors, rightMotors);

       if (RobotBase.isSimulation()){

       }

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

        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftSimRoad(), getLeftSimRoad());
        SmartDashboard.putData("Field", field);

    }

    //Updates robot's location on the field
    @Override
    public void periodic() {
        currentPose = odometry.update(navX.getRotation2d(), getLeftSimRoad(), getRightSimRoad());
        field.setRobotPose(currentPose);
        SmartDashboard.putNumber("Left Encoder", getLeftSimRoad());
        SmartDashboard.putNumber("Right Encoder", getRightSimRoad());
        SmartDashboard.putNumber("NavX Rotation", navX.getRotation2d().getDegrees());

        SmartDashboard.putNumber("Joystick 1. Axis", joy1.getRawAxis(1));
        SmartDashboard.putNumber("Joystick 4. Axis", joy1.getRawAxis(4));
    }

    public void simulationPeriodic() {

    }

    public Command driveArcade(
        DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> m_Drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
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
        resetEncoders();
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
        return navX.getYaw();
    }

}
