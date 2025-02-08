package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    private static final double WheelPerimeter = Units.inchesToMeters(4)*2*Math.PI;
    private static final double GearRatio = 1/6.75;

    public static double getTotalRoad(double encoderPosition){
      return WheelPerimeter*GearRatio*encoderPosition;
    }

    
  }
}
