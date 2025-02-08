package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DropperSubsystem extends SubsystemBase {
    private final SparkMax dropMotor;
    private final RelativeEncoder dropRelativeEncoder;

    public DropperSubsystem() {
        dropMotor = new SparkMax(4, MotorType.kBrushed);

        dropRelativeEncoder = dropMotor.getEncoder();

        dropMotor.setCANTimeout(250);

        SparkMaxConfig dropConfig = new SparkMaxConfig();
        dropConfig.voltageCompensation(10);
        dropConfig.smartCurrentLimit(60);
        dropMotor.configure(dropConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runRoller(
      DropperSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> dropMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }
  public double getDropWay() {
    return dropRelativeEncoder.getPosition();
  }
}
