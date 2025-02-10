package frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.AutoConstants;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSim {
    DriveSubsystem DriveS = new DriveSubsystem();
    Constants cons = new Constants();

    Joystick joy1 = new Joystick(0);
    
    private DifferentialDrivetrainSim driveSim = 
        DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleNEOPerSide, 
            KitbotGearing.k10p71, 
            KitbotWheelSize.kSixInch,
            null);

        private double leftAppliedVolts = 0.0;
        private double rightAppliedVolts = 0.0;
        private boolean closedLoop = false;
        private PIDController leftPID = new PIDController(0.05, 0.0, 0);
        private PIDController rightPID = new PIDController(0.05, 0.0, 0);
        private double leftFFVolts = 0.0;
        private double rightFFVolts = 0.0;

        //Encoder Sim
        private EncoderSim leftEncoderSim = new EncoderSim(DriveS.leftEncoder);
        private EncoderSim rightEncoderSim = new EncoderSim(DriveS.rightEncoder);

        private double leftRoadSim = frc.robot.Constants.OperatorConstants.getTotalRoad(leftEncoderSim.getDistance());
        private double rightRoadSim = frc.robot.Constants.OperatorConstants.getTotalRoad(rightEncoderSim.getDistance());

        //NavX Sensor
        private final ADXRS450_Gyro navX = new ADXRS450_Gyro();

        private Field2d field = new Field2d();
        private final ADXRS450_GyroSim gyroSim;
        private Pose2d currentPose;
        
        //Odometry for tracking robot's location
        private final DifferentialDriveOdometry odometry;
        private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TrackWidthMeters);
        private final PIDController xPidController = new PIDController(AutoConstants.PXController, AutoConstants.IXController, AutoConstants.DXController);
        private final PIDController yawPidController = new PIDController(AutoConstants.PYawController, AutoConstants.IYawController, AutoConstants.DYawController);

        //Datas we will use
        private double leftRoad;
        private double rightRoad;

        public DriveSim() {
            DriveS.resetSensors();

            gyroSim = new ADXRS450_GyroSim(navX);

            odometry = 
            new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(DriveS.getHeading()),
                leftEncoderSim.getDistance(), 
                rightEncoderSim.getDistance());

            field = new Field2d();
            SmartDashboard.putData("Field", field);

        }

        public Command driveArcade(
                DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
            return Commands.run(
                () -> driveSubsystem.driveArcade(driveSubsystem, xSpeed, zRotation), driveSubsystem);
        }
    
}
