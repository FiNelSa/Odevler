package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.subsystems.Drive.DriveIOSim;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  Joystick joy1 = new Joystick(0);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  DriveIOSim m_robotDriveSim = new DriveIOSim();
  DriveSubsystem m_robotDrive = new DriveSubsystem();
  DropperSubsystem m_robotDropper = new DropperSubsystem();

  UsbCamera frontCam;
  UsbCamera rearCam;

  public RobotContainer() {

    frontCam = CameraServer.startAutomaticCapture(0);
    rearCam = CameraServer.startAutomaticCapture(1);
    
    configureBindings();
    
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
