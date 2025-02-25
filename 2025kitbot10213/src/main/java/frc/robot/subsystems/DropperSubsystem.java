// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DropperConstants;;

public class DropperSubsystem extends SubsystemBase {

  private final SparkMax dropMotor = new SparkMax(DropperConstants.DropperMotorPort, MotorType.kBrushed);
  Encoder dropEncoder = new Encoder(DropperConstants.DropperMotorPort, 6, false);
  PIDController dropPidController = new PIDController(0.1, 0.0, 0.0);

  /** Creates a new DropperSubsystem. */
  public DropperSubsystem() {
    dropMotor.setCANTimeout(250);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rundropper(double forward, double reverse) {
    dropMotor.set(forward - reverse);
  }
}
