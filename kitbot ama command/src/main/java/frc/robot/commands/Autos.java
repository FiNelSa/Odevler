package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {
  public static final Command exampleAuto(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0).withTimeout(1.0);
  }
}
