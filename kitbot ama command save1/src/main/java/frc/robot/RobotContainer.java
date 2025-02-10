package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class RobotContainer {
  Joystick joy1 = new Joystick(0);

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
        new frc.robot.subsystems.Drive.DriveSim().driveArcade(
          m_robotDrive, 
          () -> joy1.getRawAxis(1), 
          () -> joy1.getRawAxis(4)));
    
      m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive, joy1));
    

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
