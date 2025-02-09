package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DropperSubsystem;

public class RobotContainer {
  CommandXboxController joy1 = new CommandXboxController(0);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  DriveSubsystem m_robotDrive = new DriveSubsystem();
  DropperSubsystem m_robotDropper = new DropperSubsystem();

  UsbCamera frontCam;
  UsbCamera rearCam;

  public RobotContainer() {

    frontCam = CameraServer.startAutomaticCapture(0);
    rearCam = CameraServer.startAutomaticCapture(1);
    
    configureBindings();

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
