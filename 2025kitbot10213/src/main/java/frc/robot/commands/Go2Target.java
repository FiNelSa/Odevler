// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Go2Target extends Command {

  private final DriveSubsystem m_DriveSubsystem;
  private Pose2d targetPose;
  private double goX;
  private double goY;
  private double oldRoad;
  private double takenRoad;
  private double line;
  private double error;
  private double speed;

  /** Creates a new Go2Target. */
  public Go2Target(DriveSubsystem driveSubsystem, Pose2d targetpose) {

    this.m_DriveSubsystem = driveSubsystem;
    this.targetPose = targetpose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 1;
    oldRoad = m_DriveSubsystem.getRoad();

    goX = targetPose.getX() - m_DriveSubsystem.getPose().getX();
    goY = targetPose.getY() - m_DriveSubsystem.getPose().getY();
    line = Math.hypot(goX, goY);
    System.out.println("line = " + line);
    error = line*2/3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    takenRoad = m_DriveSubsystem.getRoad() - oldRoad;

    m_DriveSubsystem.arcadeDrive(speed, 0);

    if(takenRoad >= error){
      speed = speed*2/3;
      error = error*3/2;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return takenRoad >= line;
  }
}
