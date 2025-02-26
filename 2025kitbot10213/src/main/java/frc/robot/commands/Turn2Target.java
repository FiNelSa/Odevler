// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turn2Target extends Command {

  DriveSubsystem driveSubsystem;
  private Pose2d targetPose;  
  private Pose2d currentPose;
  private double angle;
  private double currentAngle;

  public PIDController turnPid = new PIDController(AutoConstants.PTurnController, AutoConstants.ITurnController, AutoConstants.DTurnController);
  
  double height;
  double width;
  double goY;
  double goX;
  public double line;

  /** Creates a new Turn2Target. */
  public Turn2Target(DriveSubsystem driveSubsystem,  Pose2d targetPose) {
    this.driveSubsystem =  driveSubsystem;
    this.targetPose =  targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPid.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = driveSubsystem.getPose();

    goY = targetPose.getY() - currentPose.getY();
    goX = targetPose.getX() - currentPose.getX();

    currentAngle = driveSubsystem.getHeading();
    
    angle = Math.toDegrees(Math.atan2(goY, goX));
    
    turnPid.setSetpoint(angle);

    double output = turnPid.calculate(currentAngle);
    driveSubsystem.arcadeDrive(0, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    line = Math.sqrt(goX*goX + goY*goY);
    System.out.println("road = " + line);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return turnPid.atSetpoint();  
  }

  public double getLine() {
    return line;
  }
}
