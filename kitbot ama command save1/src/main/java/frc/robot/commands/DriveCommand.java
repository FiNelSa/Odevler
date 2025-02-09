package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    public final DriveSubsystem driveSubsystem;
    private final Joystick joy1;

    public DriveCommand(DriveSubsystem driveSubsystem, Joystick joy1) {
        this.driveSubsystem = driveSubsystem;
        this.joy1 = joy1;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveArcade(
            driveSubsystem,
             () -> -joy1.getRawAxis(1),
             () -> -joy1.getRawAxis(4));
    }
}