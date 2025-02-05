package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveSubsystem extends SubsystemBase {

    private final SparkMax leftMaster = new SparkMax(0, MotorType.kBrushed);
    private final SparkMax leftSlave = new SparkMax(1, MotorType.kBrushed);
    private final SparkMax rightMaster = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax rightSlave = new SparkMax(3, MotorType.kBrushed);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);
    
    private final DifferentialDrive m_Drive = new DifferentialDrive(leftMotors::set, rightMotors::set);

    private final RelativeEncoder lefRelativeEncoder = leftMaster.getEncoder();
    private final RelativeEncoder righRelativeEncoder = rightMaster.getEncoder();

    public void ResetEncoders() {
        lefRelativeEncoder.setPosition(0);
        righRelativeEncoder.setPosition(0);
    }

    public DriveSubsystem(){
        rightMotors.setInverted(true);
    }
}
