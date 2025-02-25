
package frc.robot.subsystems.Drive;

import com.google.flatbuffers.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
  Constants cons = new Constants();

  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(0.05, 0.0, 0);
  private PIDController rightPID = new PIDController(0.05, 0.0, 0);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts =
          leftFFVolts + leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / Units.inchesToMeters(2));
      rightAppliedVolts =
          rightFFVolts
              + leftPID.calculate(sim.getRightVelocityMetersPerSecond() / Units.inchesToMeters(2));
    }

    // Update simulation state
    sim.setInputs(
        MathUtil.clamp(leftAppliedVolts, -12.0, 12.0),
        MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.leftPositionRad = sim.getLeftPositionMeters() / Units.inchesToMeters(2);
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Units.inchesToMeters(2);
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

    inputs.rightPositionRad = sim.getRightPositionMeters() / Units.inchesToMeters(2);
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / Units.inchesToMeters(2);
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
  }
}
