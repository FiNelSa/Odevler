package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot { // TimedRobot yerine LoggedRobot kullanıyoruz
  private RobotContainer robotContainer;
  private Command m_autonomousCommand;
  private Field2d field = new Field2d();

  @Override
  public void robotInit() {
    // ✅ AdvantageKit Logger'ı başlatıyoruz
    Logger.recordMetadata("RobotName", "FRC-2025-Bot"); // Robot ismi
    Logger.addDataReceiver(new NT4Publisher()); // NetworkTables bağlantısı
    Logger.addDataReceiver(LogFileUtil.create);
    LoggedDriverStation.start();
    Logger.start();

    // Field2d objesini SmartDashboard'a ekle
    SmartDashboard.putData("Field", field);

    // RobotContainer başlat
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // WPILib komutlarını çalıştır

    // ✅ AdvantageKit için Field2d Güncellemesi
    field.setRobotPose(new Pose2d()); // Pose2d'yi güncelleyerek robot konumunu ekliyoruz
    Logger.recordOutput("RobotPose", new Pose2d());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    Logger.getInstance().disableDeterministicTimestamps(); // Simülasyon için zamanlama düzeltmesi
  }

  @Override
  public void simulationPeriodic() {}
}
