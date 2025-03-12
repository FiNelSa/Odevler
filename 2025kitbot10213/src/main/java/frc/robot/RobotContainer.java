// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.commands.DropperCommand;
import frc.robot.commands.Go2Target;
import frc.robot.commands.Turn2Target;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
@SuppressWarnings("unused")

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DropperSubsystem m_DropperSubsystem = new DropperSubsystem();

  Pose2d coralDrop = FieldConstants.CoralDrop;
  Pose2d coralTurn = FieldConstants.CoralTurn;

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final XboxController xboxController = new XboxController(OIConstants.DriverControllerPort);

  public RobotContainer() {

    NamedCommands.registerCommand("Drop Coral", new DropperCommand(1.0, 0.0, m_DropperSubsystem));
    NamedCommands.registerCommand("Turn to coral drop", new Turn2Target(m_driveSubsystem, coralDrop));
    NamedCommands.registerCommand("Turn to coral turn", new Turn2Target(m_driveSubsystem, coralTurn));
    NamedCommands.registerCommand("Go to coral drop", new Go2Target(m_driveSubsystem, coralDrop));

    boolean isCompetition = true;

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

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
            xboxController.getRightTriggerAxis(),
            xboxController.getLeftTriggerAxis(),
            m_DropperSubsystem));
  }

  //Runs the command when the button is pressed
  private void configureButtonBindings() {
    new JoystickButton(xboxController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1)));

    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new Turn2Target(m_driveSubsystem, coralDrop));
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new Go2Target(m_driveSubsystem, coralDrop));
          
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new DropperCommand(1.0, 0.0, m_DropperSubsystem));
  }

  public Command getAutonomousCommand() {
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
  
      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Could not find Example Path", null);
      return Commands.none();
    }
  }

}
