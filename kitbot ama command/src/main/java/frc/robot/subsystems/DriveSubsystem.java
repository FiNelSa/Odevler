package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DropperSubsystem;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftMaster;
    private final SparkMax leftSlave;
    private final SparkMax rightMaster;
    private final SparkMax rightSlave;

    private final DifferentialDrive m_Drive;
    
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d()); 
    private final DifferentialDriveOdometry odometry;

    public DriveSubsystem() {

        leftMaster = new SparkMax(0, MotorType.kBrushed);
        leftSlave = new SparkMax(1, MotorType.kBrushed);
        rightMaster = new SparkMax(2, MotorType.kBrushed);
        rightSlave = new SparkMax(3, MotorType.kBrushed);

        final RelativeEncoder leftRelativeEncoder = leftMaster.getEncoder();    
        final RelativeEncoder rightRelativeEncoder = rightMaster.getEncoder();

        Rotation2d gyroAngle = new Rotation2d((rightRelativeEncoder.getPosition()-leftRelativeEncoder.getPosition())/0.64);
        
        odometry = new DifferentialDriveOdometry(gyroAngle, leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition());

        double lRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftRelativeEncoder.getPosition());
        double rRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightRelativeEncoder.getPosition());

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

    public Command driveArcade(
        DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> m_Drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
    }

    public Pose2d getPose() {
        return pose;
    }
}
