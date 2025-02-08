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
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private final ShuffleboardTab fieldTab;

  private final CommandXboxController joy1 = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final DropperSubsystem m_robotDropper = new DropperSubsystem();

  public RobotContainer() {
    fieldTab = Shuffleboard.getTab("Field");

    configureBindings();

    fieldTab.add("Left Speed", m_robotDrive.getLeftRoad()).withSize(2, 1).withPosition(0, 0);
    fieldTab.add("Right Speed", m_robotDrive.getRightRoad()).withSize(2, 1).withPosition(2, 0);

    fieldTab.add("Roller Speed", m_robotDropper.getDropWay()).withSize(2, 1).withPosition(0, 1);

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
