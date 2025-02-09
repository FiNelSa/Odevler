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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private final RelativeEncoder leftRelativeEncoder;
    private final RelativeEncoder rightRelativeEncoder;

    //NavX Sensor
    private final AHRS navX = new AHRS(Port.kMXP);

    //Odometry for tracking robot's location
    private final DifferentialDriveOdometry odometry;
    private Field2d field = new Field2d();
    private Pose2d currentPose;

    //Datas we will use
    private double leftRoad;
    private double rightRoad;

    //Supresses useless warnings
    @SuppressWarnings("removal")
    public DriveSubsystem() {

        leftMaster = new SparkMax(0, MotorType.kBrushed);
        leftSlave = new SparkMax(1, MotorType.kBrushed);
        rightMaster = new SparkMax(2, MotorType.kBrushed);
        rightSlave = new SparkMax(3, MotorType.kBrushed);

        leftRelativeEncoder = leftMaster.getEncoder();    
        rightRelativeEncoder = rightMaster.getEncoder();

        leftRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftRelativeEncoder.getPosition());
        rightRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightRelativeEncoder.getPosition());

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

        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
        SmartDashboard.putData("Field", field);

    }

    //Updates robot's location on the field
    @Override
    public void periodic() {
        
            leftRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftRelativeEncoder.getPosition());
            rightRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightRelativeEncoder.getPosition());

            currentPose = odometry.update(navX.getRotation2d(), leftRoad, rightRoad);
            field.setRobotPose(currentPose);
            SmartDashboard.putNumber("Left Encoder", leftRoad);
            SmartDashboard.putNumber("Right Encoder", rightRoad);
            SmartDashboard.putNumber("NavX Rotation", navX.getRotation2d().getDegrees());

            SmartDashboard.putNumber("Joystick X", joy1.getRawAxis(1));
            SmartDashboard.putNumber("Joystick Y", joy1.getRawAxis(4));

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

    public void resetEncoders() {
        leftRelativeEncoder.setPosition(0);
        rightRelativeEncoder.setPosition(0);
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
