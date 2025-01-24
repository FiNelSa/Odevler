package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import java.io.ObjectInputFilter.Config;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.Joystick;

public class DriveSubsystem extends SubsystemBase {

  private Joystick joy1 = new Joystick(0);
  
  private SparkMax leftMaster = new SparkMax(0, MotorType.kBrushed);
  private SparkMax leftSlave = new SparkMax(1, MotorType.kBrushed);

  private SparkMax rightMaster = new SparkMax(2, MotorType.kBrushed);
  private SparkMax rightSlave = new SparkMax(3, MotorType.kBrushed);

  private final RelativeEncoder leftEncoder = leftMaster.getEncoder();
  private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

  private final double kEncoderTick2Meter = leftEncoder.getPosition()*(Math.PI)*7.12/4.23;

  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftMaster, leftSlave);
  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightMaster,rightSlave);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  public DriveSubsystem() {
    rightMotorControllerGroup.setInverted(true);
    leftMotorControllerGroup.setInverted(false);
  }


  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
        });
  }


  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    
  }

  public void setMotors(double leftSpeed, double rightSpeed){
    leftMaster.set(leftSpeed);
    rightMaster.set(-rightSpeed);
  }
}
