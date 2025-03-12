// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DropperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DropperCommand extends Command {

  private final Double forward;
  private final Double reverse;
  private double road;
  private double takenRoad;
  private double oldRoad;

  private DropperSubsystem m_DropperSubsystem;

  /** Creates a new DropperCommand. */
  public DropperCommand(Double forward, Double reverse, DropperSubsystem dropperSubsystem) {

    this.forward = forward;
    this.reverse = reverse;
    this.m_DropperSubsystem = dropperSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_DropperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldRoad = m_DropperSubsystem.getRoad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    road = m_DropperSubsystem.getRoad();
    takenRoad = road - oldRoad;
    m_DropperSubsystem.rundropper(forward, reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (takenRoad == forward && -takenRoad == reverse);
  }
}
