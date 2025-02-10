package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    private static final double WheelPerimeter = Units.inchesToMeters(4)*2*Math.PI;
    private static final double GearRatio = 1/6.75;

    public static double getTotalRoad(double encoderPosition){

      return WheelPerimeter*GearRatio*encoderPosition/DriveConstants.EncoderCPR;

    }

    public static final class DriveConstants {
      public static final int LeftMotor1Port = 0;
      public static final int LeftMotor2Port = 1;
      public static final int RightMotor1Port = 2;
      public static final int RightMotor2Port = 3;

      public static final int[] LeftEncoderPorts = new int[] {0, 1};
      public static final int[] RightEncoderPorts = new int[] {2, 3};

      public static final boolean LeftEncoderInverted = false;
      public static final boolean RightEncoderInverted = false;

      public static final double TrackWidthMeters = 0.64;
      public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
        new DifferentialDriveKinematics(TrackWidthMeters);
      
      public static final int EncoderCPR = 1024;
      public static final double WheelDiameterMeters = Units.inchesToMeters(4)*2;
      public static final double EncoderDistancePerPulse = 
        (WheelDiameterMeters*Math.PI)/EncoderCPR;

      public static final boolean GyroInverted = true;

      public static final double sVolts = 0.22;
      public static final double vVoltSecondsPerMeter = 1.98;
      public static final double aVoltSecondsSquaredPerMeter = 0.2;

      public static final double vVoltSecondsPerRadian = 1.5;
      public static final double aVoltSecondsSquaredPerRadian = 0.3;
    
      public static final LinearSystem<N2, N2, N2> DriveTrainPlant = 
        LinearSystemId.identifyDrivetrainSystem(
          vVoltSecondsPerMeter,
          aVoltSecondsSquaredPerMeter,
          vVoltSecondsPerRadian,
          aVoltSecondsSquaredPerRadian);

      public static final DCMotor DriveGearbox = DCMotor.getNEO(2);
      public static final double DriveGearing = 8;

      public static final double PDriveVel = 0.5;
    
    }

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;

    }

    public static final class AutoConstants {
      public static final double MaxSpeedMetersPerSecond = 3;
      public static final double MaxAccelerationMetersPerSecondSquare = 3;

      public static final double RamseteB = 2.3;
      public static final double RamseteZeta = 1.4; 
      public static final double PXController = 2;
      public static double IXController = 0.0001;
      public static double DXController = 0.00001;

      public static final double PYawController = 1.7;
      public static final double IYawController = 0.0001;
      public static double DYawController = 0.00001;

      public static final double PTurnController = 0.15;
      public static final double ITurnController = 0.000009;
      public static double DTurnController = 0.015;

    }

    public static final class FieldConstants {
      public static final Pose2d BLUE_SUB_WOOFER = new Pose2d(new Translation2d(0, 5.60), new Rotation2d(Math.toRadians(180)));
      public static final Pose2d RED_SUB_WOOFER = new Pose2d(new Translation2d(16.5, 5.60), new Rotation2d(Math.toRadians(0)));
    }

  }
}
