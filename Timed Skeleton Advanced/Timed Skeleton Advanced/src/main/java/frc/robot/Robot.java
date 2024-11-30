// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private CANSparkMax leftMaster = new CANSparkMax(0, MotorType.kBrushed);
  private CANSparkMax leftSlave = new CANSparkMax(1, MotorType.kBrushed);
  private CANSparkMax rightMaster = new CANSparkMax(2, MotorType.kBrushed);
  private CANSparkMax rightSlave = new CANSparkMax(3, MotorType.kBrushed);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  private Joystick joy1 = new Joystick(0);

  private double startTime;

  private final double kDriveTick2Feet = 1.0 / 4096 * 6* Math.PI/ 12;

  @Override
  public void robotInit() {
    startTime = Timer.getFPGATimestamp();

    leftMaster.setInverted(true);
    rightMaster.setInverted(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(leftMaster.getInverted());
    rightSlave.setInverted(rightMaster.getInverted());

    RelativeEncoder leftEncoder = leftMaster.getEncoder();
    leftEncoder.setPositionConversionFactor(0);
    leftEncoder.setVelocityConversionFactor(-10);

    RelativeEncoder rightEncoder = rightMaster.getEncoder();
    rightEncoder.setPositionConversionFactor(0);
    rightEncoder.setVelocityConversionFactor(10); 

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    drive.setDeadband(0.05);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    RelativeEncoder leftEncoder = leftMaster.getEncoder();
    leftEncoder.setPositionConversionFactor(0);
    leftEncoder.setVelocityConversionFactor(-10);

    RelativeEncoder rightEncoder = rightMaster.getEncoder();
    rightEncoder.setPositionConversionFactor(0);
    rightEncoder.setVelocityConversionFactor(10); 
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getRawAxis(1)*0.6;
    double turn = joy1.getRawAxis(4)*0.3;
    /* 
    if(Math.abs(speed) < 0.05){
      speed = 0;
    }

    if(Math.abs(turn) < 0.05){
      turn = 0;
    }
    */
    drive.arcadeDrive(speed, turn);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
