package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.Encoder;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax intakeLeftMotor = new SparkMax(6, MotorType.kBrushed);
    private final SparkMax intakeRightMotor = new SparkMax(7, MotorType.kBrushed);


    public IntakeSubsystem() {
    }

    public void setPosition(boolean open){
        if(open){
            intakeLeftMotor.set(-1);
            intakeRightMotor.set(-1);
        } else {
            intakeLeftMotor.set(1);
            intakeRightMotor.set(1);
        }
    }
}
