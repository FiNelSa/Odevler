package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveIOSim;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveCommand extends Command {

    public final DriveSubsystem driveSubsystem;
    private final Joystick joy1;

    public DriveCommand(DriveSubsystem m_robotDrive, Joystick joy1) {
        this.driveSubsystem = m_robotDrive;
        this.joy1 = joy1;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        driveSubsystem.driveArcade(
            driveSubsystem,
             () -> -joy1.getRawAxis(1),
             () -> -joy1.getRawAxis(4));
    }
}