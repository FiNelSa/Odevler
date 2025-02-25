// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.commands.Drive;
import frc.robot.commands.DropperCommand;
import frc.robot.commands.Go2Target;
import frc.robot.commands.Turn2Target;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DropperSubsystem m_DropperSubsystem = new DropperSubsystem();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public double calculateDistanceToTarget(Pose2d targetPose) {
    return m_driveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation());
  }

  private final XboxController xboxController =
      new XboxController(OIConstants.DriverControllerPort);

  public RobotContainer() {
    //autoChooser = AutoBuilder.buildAutoChooser("New Auto");

    configureButtonBindings();

    //Default command for the drive subsystem
    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
            () ->
                m_driveSubsystem.arcadeDrive(
                    -xboxController.getRawAxis(1), -xboxController.getRawAxis(4)),
                    m_driveSubsystem));

    m_DropperSubsystem.setDefaultCommand(
      new DropperCommand(
            () -> xboxController.getRightTriggerAxis(),
            () -> xboxController.getLeftTriggerAxis(), 
            m_DropperSubsystem));
  }

  //Runs the command when the button is pressed
  private void configureButtonBindings() {
    new JoystickButton(xboxController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1)));
 
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new Drive(m_driveSubsystem, 5.0));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new Turn2Target(m_driveSubsystem, DriveConstants.targetPose));     
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new Go2Target(m_driveSubsystem, DriveConstants.targetPose));
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
