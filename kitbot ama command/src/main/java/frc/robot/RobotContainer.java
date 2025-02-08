package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DropperSubsystem;

public class RobotContainer {
  private final ShuffleboardTab autoTab;

  private final CommandXboxController joy1 = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final DropperSubsystem m_robotDropper = new DropperSubsystem();

  public RobotContainer() {
    autoTab = Shuffleboard.getTab("Auto");

    configureBindings();
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(m_robotDrive));
    SmartDashboard.putData("Auto Mode", autoChooser);

    m_robotDrive.setDefaultCommand(
        m_robotDrive.driveArcade(
            m_robotDrive,
             () -> -joy1.getRawAxis(1),
             () -> -joy1.getRawAxis(4)));

    m_robotDropper.setDefaultCommand(
        m_robotDropper.runRoller(
            m_robotDropper,
            () -> joy1.getRawAxis(3),
            () -> joy1.getRawAxis(2)));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
