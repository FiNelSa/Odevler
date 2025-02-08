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
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftMaster;
    private final SparkMax leftSlave;
    private final SparkMax rightMaster;
    private final SparkMax rightSlave;

    private final DifferentialDrive m_Drive;

    private final RelativeEncoder leftRelativeEncoder;
    private final RelativeEncoder rightRelativeEncoder;
    private double leftRoad;
    private double rightRoad;
    
    private Rotation2d gyroAngle;
    private final ShuffleboardTab driveTab;

    private Pose2d pose = new Pose2d();

    public DriveSubsystem() {
        Constants cons = new Constants();

        leftMaster = new SparkMax(0, MotorType.kBrushed);
        leftSlave = new SparkMax(1, MotorType.kBrushed);
        rightMaster = new SparkMax(2, MotorType.kBrushed);
        rightSlave = new SparkMax(3, MotorType.kBrushed);

        leftRelativeEncoder = leftMaster.getEncoder();    
        rightRelativeEncoder = rightMaster.getEncoder();

        leftRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(leftRelativeEncoder.getPosition());
        rightRoad = frc.robot.Constants.OperatorConstants.getTotalRoad(rightRelativeEncoder.getPosition());

        gyroAngle = new Rotation2d((rightRoad - leftRoad) / 0.64);

        driveTab = Shuffleboard.getTab("Drive");

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive/Left Encoder", leftRelativeEncoder.getPosition());
        SmartDashboard.putNumber("Drive/Right Encoder", rightRelativeEncoder.getPosition());

        SmartDashboard.putNumber("Drive/Left Speed", leftRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("Drive/Right Speed", rightRelativeEncoder.getVelocity());
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
    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }
}
