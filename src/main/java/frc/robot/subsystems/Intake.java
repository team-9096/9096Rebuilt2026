package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkMax pivot;
    private SparkMax intake;

    public Intake() {
        pivot = new SparkMax(Constants.MechanismConstants.INTAKE_PIVOT, MotorType.kBrushless);
        intake = new SparkMax(Constants.MechanismConstants.INTAKE_MOTOR, MotorType.kBrushless);
    }
    
    

}
