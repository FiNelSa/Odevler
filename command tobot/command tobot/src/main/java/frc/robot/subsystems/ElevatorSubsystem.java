package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorSubsystem {
    
    private final SparkMax elevatorMotor1 = new SparkMax(4, MotorType.kBrushed);
    private final SparkMax elevatorMotor2 = new SparkMax(5, MotorType.kBrushed);
    private final Encoder EleEncoder1 = new Encoder(4, 5);
    private final Encoder EleEncoder2 = new Encoder(6, 7);
    private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

    public double getEncoderMeters1() {
        return EleEncoder1.get() * kEncoderTick2Meter;
    }
    public double getEncoderMeters2() {
        return EleEncoder2.get() * kEncoderTick2Meter;
    }
    public void setMotors(double speed1, double speed2){
        elevatorMotor1.set(speed1);
        elevatorMotor2.set(speed2);
    }
}
